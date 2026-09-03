#ifndef TIMEDSERIAL_UART_DRIVER_H
#define TIMEDSERIAL_UART_DRIVER_H

#include "common.hpp"
#include "../serial_interface.hpp"

#include "xuc_protocol.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <utility>

class UartDriver : public SerialInterface
{
private:
    AttitudeCallback attitude_cb_;
    RobotStatusCallback status_cb_;

    const std::string m_device_name;
    const std::string m_attitude_device_name;
    int fd_ = -1;
    int attitude_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::thread attitude_receive_thread_;
    std::mutex write_mutex_;
    std::mutex attitude_mutex_;

    // Hero status 28B now carries roll/pitch/yaw from imu_pantile, no separate HI91 needed.
    Attitude latest_attitude_{};
    std::atomic<bool> attitude_seen_{false};

public:
    explicit UartDriver(const std::string &device_name,
                        const std::string &attitude_device_name = {});

    void set_attitude_callback(AttitudeCallback cb) override { attitude_cb_ = std::move(cb); }
    void set_robot_status_callback(RobotStatusCallback cb) override { status_cb_ = std::move(cb); }

    bool init() override;
    void start() override;
    void close() override;

    void transmit_cmd(float yaw,
                      float yaw_spd,
                      float pitch,
                      float pitch_spd,
                      float yaw_acc,
                      float pitch_acc,
                      float dist,
                      std::uint8_t shoot = 0,
                      std::uint8_t target_id = 0) override;

    bool is_open() const { return fd_ >= 0; }
    ~UartDriver() override { close(); }

private:
    static bool configure_port(int fd, speed_t baud);
    void receive_loop();
    void attitude_receive_loop();
    bool enable_hi91_output();
    void publish_attitude(float yaw, float pitch, float roll, std::uint32_t timestamp_ms);
    void publish_status(const xuc_protocol::StatusPacket &packet);
};

#endif
