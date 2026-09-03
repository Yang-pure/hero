#ifndef TIMEDSERIAL_XUC_PROTOCOL_HPP
#define TIMEDSERIAL_XUC_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace xuc_protocol
{
// Hero firmware (Yang-pure/hero) frames: 0x5A status 28B, 0xA5 control 29B
// See /home/linyudong/.agent-isolation/worktrees/8f13073e5b891225/shared/autoaim-deploy/HERO_COMMS_STATUS.md
constexpr std::uint8_t HERO_STATUS_HEAD = 0x5A;
constexpr std::uint8_t HERO_CONTROL_HEAD = 0xA5;
constexpr std::size_t HERO_STATUS_FRAME_SIZE = 28;
constexpr std::size_t HERO_CONTROL_FRAME_SIZE = 29;
// Legacy SP frames kept for reference but not used on hero
constexpr std::uint8_t FRAME_HEAD_0 = 'S';
constexpr std::uint8_t FRAME_HEAD_1 = 'P';
constexpr std::size_t STATUS_FRAME_SIZE = HERO_STATUS_FRAME_SIZE;
constexpr std::size_t CONTROL_FRAME_SIZE = HERO_CONTROL_FRAME_SIZE;

#pragma pack(push, 1)
// Hero status: 0:0x5A, 1:flags(bit0 detect_color 0=red1=blue, bit1 reset_tracker, bit2-7=15), 2..13 roll/pitch/yaw, 14..25 aim_xyz, 26..27 crc16 over [0..26)
struct StatusPacket
{
    std::uint8_t head;      // 0x5A
    std::uint8_t flags;     // bit0 detect_color, bit1 reset_tracker
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    std::uint16_t crc16;
};

// Hero control: 0:0xA5, 1..4 pitch(deg), 5..8 yaw, 9..12 yaw_diff, 13..16 pitch_diff(deg), 17..20 distance, 21 fireadvice, 22..24 pad, 25..28 v_y (29B, no CRC per firmware Decode)
struct ControlPacket
{
    std::uint8_t head;      // 0xA5
    float pitch;
    float yaw;
    float yaw_diff;
    float pitch_diff;
    float distance;
    std::uint8_t fireadvice;
    std::uint8_t pad[3];
    float v_y;
};
#pragma pack(pop)

static_assert(sizeof(StatusPacket) == STATUS_FRAME_SIZE);
static_assert(sizeof(ControlPacket) == CONTROL_FRAME_SIZE);

inline std::uint16_t crc16(const std::uint8_t *data, std::size_t length)
{
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 1U) ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                              : static_cast<std::uint16_t>(crc >> 1U);
    }
    return crc;
}

inline void append_crc16(std::uint8_t *frame, std::size_t length)
{
    const std::uint16_t value = crc16(frame, length - 2);
    frame[length - 2] = static_cast<std::uint8_t>(value & 0xffU);
    frame[length - 1] = static_cast<std::uint8_t>(value >> 8U);
}

inline bool verify_crc16(const std::uint8_t *frame, std::size_t length)
{
    if (frame == nullptr || length < 4)
        return false;

    const std::uint16_t expected = crc16(frame, length - 2);
    const std::uint16_t actual = static_cast<std::uint16_t>(frame[length - 2]) |
                                 static_cast<std::uint16_t>(frame[length - 1] << 8U);
    return expected == actual;
}

inline bool has_head(const std::uint8_t *frame)
{
    return frame != nullptr && frame[0] == HERO_STATUS_HEAD;
}

inline bool is_hero_control_head(const std::uint8_t *frame)
{
    return frame != nullptr && frame[0] == HERO_CONTROL_HEAD;
}

// Hero control: pitch/yaw in deg as per firmware (pitch *PI/180 inside), yaw_diff/pitch_diff, distance, fireadvice, v_y (29B, firmware does not check CRC)
inline void encode_control(std::uint8_t *frame,
                           float pitch_deg,
                           float yaw,
                           float yaw_diff,
                           float pitch_diff_deg,
                           float distance,
                           std::uint8_t fireadvice,
                           float v_y)
{
    ControlPacket packet{};
    packet.head = HERO_CONTROL_HEAD;
    packet.pitch = pitch_deg;
    packet.yaw = yaw;
    packet.yaw_diff = yaw_diff;
    packet.pitch_diff = pitch_diff_deg;
    packet.distance = distance;
    packet.fireadvice = fireadvice & 0x01U;
    packet.pad[0] = 0; packet.pad[1] = 0; packet.pad[2] = 0;
    packet.v_y = v_y;
    std::memcpy(frame, &packet, sizeof(packet));
    // No CRC per hero firmware (Decode does not verify); if future firmware enables CRC, append here
}

// Legacy SP encoder kept for compatibility (unused on hero)
inline void encode_control(std::uint8_t *frame,
                           std::uint8_t control,
                           std::uint8_t shoot,
                           float yaw,
                           float pitch,
                           float imu_pitch,
                           float imu_yaw)
{
    // Map legacy SP fields to hero fields: control-> distance>0 ? track, shoot->fireadvice
    const float distance = (control != 0) ? 1.0f : 0.0f;
    encode_control(frame, pitch, yaw, 0.0f, 0.0f, distance, shoot, 0.0f);
}

inline StatusPacket decode_status(const std::uint8_t *frame)
{
    StatusPacket packet{};
    std::memcpy(&packet, frame, sizeof(packet));
    return packet;
}
}

#endif
