#include "uart_driver.hpp"

#include "packet.hpp"

#include <array>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace
{
constexpr speed_t XUC_BAUD = B460800;
constexpr speed_t HI91_BAUD = B115200;
constexpr float DEG_TO_RAD = 0.01745329251994329577f;
constexpr float RAD_TO_DEG = 57.29577951308232087679f;

bool is_disabled_port(const std::string &port)
{
    return port.empty() || port == "None" || port == "none";
}

std::uint16_t hi91_crc16_update(std::uint16_t crc,
                                const std::uint8_t *data,
                                std::size_t length)
{
    for (std::size_t index = 0; index < length; ++index)
    {
        crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 0x8000U)
                      ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                      : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

float read_float_le(const std::uint8_t *data)
{
    float value{};
    std::memcpy(&value, data, sizeof(value));
    return value;
}

std::uint32_t read_u32_le(const std::uint8_t *data)
{
    std::uint32_t value{};
    std::memcpy(&value, data, sizeof(value));
    return value;
}

ProgramMode decode_program_mode(std::uint8_t mode)
{
    switch (mode)
    {
    case 1:
        return ProgramMode::AUTO_AIM;
    case 2:
        return ProgramMode::SMALL_ENERGY;
    case 3:
        return ProgramMode::BIG_ENERGY;
    default:
        return ProgramMode::IDLE;
    }
}
}

UartDriver::UartDriver(const std::string &device_name,
                       const std::string &attitude_device_name)
    : m_device_name(device_name),
      m_attitude_device_name(attitude_device_name)
{
}

bool UartDriver::configure_port(int fd, speed_t baud)
{
    struct termios options
    {
    };
    if (tcgetattr(fd, &options) != 0)
        return false;

    cfmakeraw(&options);
    if (cfsetispeed(&options, baud) != 0 || cfsetospeed(&options, baud) != 0)
        return false;

    options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    options.c_cflag &= static_cast<tcflag_t>(~PARENB);
    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    return tcsetattr(fd, TCSANOW, &options) == 0;
}

bool UartDriver::init()
{
    close();
    attitude_seen_.store(false);
    const int fd = ::open(m_device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        LOGE_S("[UART][XUC] cannot open %s: %s", m_device_name.c_str(), std::strerror(errno));
        return false;
    }

    if (!configure_port(fd, XUC_BAUD))
    {
        LOGE_S("[UART][XUC] cannot configure %s: %s", m_device_name.c_str(), std::strerror(errno));
        ::close(fd);
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    LOGM_S("[UART][XUC] opened %s at 460800 baud, hero 0x5A(28B)/0xA5(29B) CRC16 protocol", m_device_name.c_str());

    if (!is_disabled_port(m_attitude_device_name))
    {
        const int attitude_fd = ::open(m_attitude_device_name.c_str(),
                                       O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (attitude_fd < 0)
        {
            LOGE_S("[UART][HI91] cannot open %s: %s",
                   m_attitude_device_name.c_str(), std::strerror(errno));
            close();
            return false;
        }

        if (!configure_port(attitude_fd, HI91_BAUD))
        {
            LOGE_S("[UART][HI91] cannot configure %s: %s",
                   m_attitude_device_name.c_str(), std::strerror(errno));
            ::close(attitude_fd);
            close();
            return false;
        }

        tcflush(attitude_fd, TCIOFLUSH);
        attitude_fd_ = attitude_fd;
        LOGM_S("[UART][HI91] opened %s at 115200 baud, 5A A5/CRC-CCITT protocol",
               m_attitude_device_name.c_str());
    }

    return true;
}

void UartDriver::start()
{
    if (fd_ < 0 || running_.exchange(true))
        return;

    receive_thread_ = std::thread(&UartDriver::receive_loop, this);
    if (attitude_fd_ >= 0)
    {
        if (!enable_hi91_output())
        {
            LOGW_S("[UART][HI91] could not send LOG ENABLE; waiting for passive HI91 frames");
        }
        attitude_receive_thread_ = std::thread(&UartDriver::attitude_receive_loop, this);
    }
    else
    {
        LOGM_S("[UART][XUC] hero mode: attitude from 0x5A status (no HI91), control remains disabled until first 28B status");
    }
}

void UartDriver::close()
{
    running_.store(false);
    if (receive_thread_.joinable())
        receive_thread_.join();
    if (attitude_receive_thread_.joinable())
        attitude_receive_thread_.join();

    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
    if (attitude_fd_ >= 0)
    {
        ::close(attitude_fd_);
        attitude_fd_ = -1;
    }
}

void UartDriver::publish_status(const xuc_protocol::StatusPacket &packet)
{
    // Hero status has no mode/robot_id/bullet_speed: use flags for color, defaults for rest
    RobotStatus status{};
    // flags bit0 detect_color 0=red 1=blue ( = !own_color )
    const bool detect_blue = (packet.flags & 0x01U) != 0;
    status.enemy_color = detect_blue ? EnemyColor::BLUE : EnemyColor::RED;
    status.program_mode = ProgramMode::AUTO_AIM;
    status.robot_speed_mps = INF_BALL_SPEED;
    // aim_x/y/z currently 0, reserved for future

    if (status_cb_)
        status_cb_(status);
}

void UartDriver::receive_loop()
{
    // Hero: 28B status 0x5A, contains roll/pitch/yaw from imu_pantile -> publish attitude directly, no HI91 needed
    std::vector<std::uint8_t> buffer;
    buffer.reserve(128);
    std::array<std::uint8_t, 64> chunk{};

    while (running_.load())
    {
        const ssize_t count = ::read(fd_, chunk.data(), chunk.size());
        if (count > 0)
        {
            buffer.insert(buffer.end(), chunk.begin(), chunk.begin() + count);

            while (buffer.size() >= 1)
            {
                if (buffer[0] != xuc_protocol::HERO_STATUS_HEAD)
                {
                    buffer.erase(buffer.begin());
                    continue;
                }

                if (buffer.size() < xuc_protocol::STATUS_FRAME_SIZE)
                    break;

                // Hero flags high 6 bits should be 15 (reserved), filter false 0x5A in payload
                if ((buffer[1] & 0xFCU) != 0x3CU) // 0b111100 = 15<<2
                {
                    buffer.erase(buffer.begin());
                    continue;
                }

                if (!xuc_protocol::verify_crc16(buffer.data(), xuc_protocol::STATUS_FRAME_SIZE))
                {
                    buffer.erase(buffer.begin());
                    continue;
                }

                const auto packet = xuc_protocol::decode_status(buffer.data());
                // Publish attitude from hero imu_pantile
                if (std::isfinite(packet.roll) && std::isfinite(packet.pitch) && std::isfinite(packet.yaw))
                {
                    // Hero yaw/pitch/roll are in degrees from imu_pantile.GetAngle*()
                    publish_attitude(packet.yaw, packet.pitch, packet.roll, 0);
                }
                publish_status(packet);
                buffer.erase(buffer.begin(),
                             buffer.begin() + xuc_protocol::STATUS_FRAME_SIZE);
            }

            if (buffer.size() > 2 * xuc_protocol::STATUS_FRAME_SIZE)
                buffer.erase(buffer.begin(), buffer.end() - xuc_protocol::STATUS_FRAME_SIZE);
        }
        else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        {
            LOGE_S("[UART][XUC] read failed on %s: %s", m_device_name.c_str(), std::strerror(errno));
            break;
        }
        else
        {
            usleep(1000);
        }
    }
}

bool UartDriver::enable_hi91_output()
{
    static constexpr std::array<std::uint8_t, 12> command{
        {'L', 'O', 'G', ' ', 'E', 'N', 'A', 'B', 'L', 'E', '\r', '\n'}};
    std::size_t offset = 0;
    while (offset < command.size())
    {
        const ssize_t written = ::write(attitude_fd_, command.data() + offset,
                                        command.size() - offset);
        if (written > 0)
        {
            offset += static_cast<std::size_t>(written);
        }
        else if (written < 0 && errno == EINTR)
        {
            continue;
        }
        else
        {
            return false;
        }
    }
    LOGM_S("[UART][HI91] sent LOG ENABLE to %s", m_attitude_device_name.c_str());
    return true;
}

void UartDriver::publish_attitude(float yaw,
                                  float pitch,
                                  float roll,
                                  std::uint32_t timestamp_ms)
{
    if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(roll))
        return;

    const Attitude attitude(yaw, pitch, roll);
    {
        std::lock_guard<std::mutex> lock(attitude_mutex_);
        latest_attitude_ = attitude;
    }

    if (!attitude_seen_.exchange(true))
    {
        LOGM_S("[UART][HERO] first pose received: yaw=%.3f pitch=%.3f roll=%.3f timestamp=%u ms",
               yaw, pitch, roll, timestamp_ms);
    }
    if (attitude_cb_)
        attitude_cb_(attitude);
}

void UartDriver::attitude_receive_loop()
{
    static constexpr std::array<std::uint8_t, 2> sync{{0x5a, 0xa5}};
    std::vector<std::uint8_t> buffer;
    buffer.reserve(4096);
    std::array<std::uint8_t, 512> chunk{};

    while (running_.load())
    {
        const ssize_t count = ::read(attitude_fd_, chunk.data(), chunk.size());
        if (count > 0)
        {
            buffer.insert(buffer.end(), chunk.begin(), chunk.begin() + count);

            while (buffer.size() >= 6)
            {
                const auto start = std::search(buffer.begin(), buffer.end(),
                                               sync.begin(), sync.end());
                if (start == buffer.end())
                {
                    if (buffer.back() == sync[0])
                        buffer.erase(buffer.begin(), buffer.end() - 1);
                    else
                        buffer.clear();
                    break;
                }
                if (start != buffer.begin())
                    buffer.erase(buffer.begin(), start);
                if (buffer.size() < 6)
                    break;

                const std::size_t payload_length =
                    static_cast<std::size_t>(buffer[2]) |
                    (static_cast<std::size_t>(buffer[3]) << 8U);
                if (payload_length == 0 || payload_length > 4096)
                {
                    buffer.erase(buffer.begin());
                    continue;
                }

                const std::size_t frame_length = 6 + payload_length;
                if (buffer.size() < frame_length)
                    break;

                const auto wire_crc = static_cast<std::uint16_t>(buffer[4]) |
                                      (static_cast<std::uint16_t>(buffer[5]) << 8U);
                auto calculated_crc = hi91_crc16_update(0, buffer.data(), 4);
                calculated_crc = hi91_crc16_update(calculated_crc,
                                                   buffer.data() + 6,
                                                   payload_length);
                if (wire_crc != calculated_crc)
                {
                    buffer.erase(buffer.begin());
                    continue;
                }

                const auto *payload = buffer.data() + 6;
                if (payload[0] == 0x91 && payload_length >= 76)
                {
                    publish_attitude(read_float_le(payload + 56),
                                     read_float_le(payload + 52),
                                     read_float_le(payload + 48),
                                     read_u32_le(payload + 8));
                }
                buffer.erase(buffer.begin(), buffer.begin() + frame_length);
            }

            if (buffer.size() > 8192)
                buffer.erase(buffer.begin(), buffer.end() - 4096);
        }
        else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        {
            LOGE_S("[UART][HI91] read failed on %s: %s",
                   m_attitude_device_name.c_str(), std::strerror(errno));
            break;
        }
        else
        {
            usleep(1000);
        }
    }
}

void UartDriver::transmit_cmd(float yaw,
                              float yaw_spd,
                              float pitch,
                              float pitch_spd,
                              float yaw_acc,
                              float pitch_acc,
                              float dist,
                              std::uint8_t shoot,
                              std::uint8_t target_id)
{
    (void)yaw_acc;
    (void)pitch_acc;
    (void)target_id;

    if (fd_ < 0)
        return;

    const bool control_enabled = attitude_seen_.load();
    // Hero control: distance>0 for 50 frames -> track_flag; fireadvice kept 0 for safety (remote fire disabled)
    const float distance = control_enabled && std::isfinite(dist) && dist > 0.1f ? dist : 0.0f;
    const std::uint8_t fireadvice = 0; // keep 0, do not use shoot param until safety gate approved (HERO_COMMS_STATUS.md 5.3)
    (void)shoot;
    // yaw_diff/pitch_diff from planner speeds
    const float yaw_diff = std::isfinite(yaw_spd) ? yaw_spd : 0.0f;
    const float pitch_diff = std::isfinite(pitch_spd) ? pitch_spd : 0.0f;
    // v_y reserved, keep 0
    const float v_y = 0.0f;

    std::array<std::uint8_t, xuc_protocol::CONTROL_FRAME_SIZE> frame{};

    // Planner already outputs degrees: send.yaw_angle/send.pitch_angle = (plan.target - attitude)/M_PI*180 (planner_submodule.cpp:126,129)
    xuc_protocol::encode_control(frame.data(),
                                 pitch,        // deg, firmware *PI/180
                                 yaw,          // deg, firmware no conversion (keep as deg)
                                 yaw_diff,     // deg/s (planner already deg)
                                 pitch_diff,   // deg/s
                                 distance,
                                 fireadvice,
                                 v_y);

    std::lock_guard<std::mutex> lock(write_mutex_);
    std::size_t offset = 0;
    while (offset < frame.size())
    {
        const ssize_t written = ::write(fd_, frame.data() + offset, frame.size() - offset);
        if (written > 0)
        {
            offset += static_cast<std::size_t>(written);
        }
        else if (written < 0 && errno == EINTR)
        {
            continue;
        }
        else
        {
            LOGE_S("[UART][XUC] write failed on %s: %s", m_device_name.c_str(), std::strerror(errno));
            break;
        }
    }
}
