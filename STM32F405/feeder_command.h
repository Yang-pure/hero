#pragma once
#include <cstdint>

// 纯输入策略，不依赖HAL/RTOS；主机测试直接复用本逻辑。
// RC新帧→双DOWN和回中解锁→时效检查→摇杆比例映射；此处不写电流、不计算PID。
// 所有速度都是电机反馈RPM，不是减速箱输出轴/拨弹盘RPM。
struct FeederCommand
{
    enum StopReason : uint8_t
    {
        RUNNING = 0, NOT_FIRE = 1, WAIT_RC = 2, RC_TIMEOUT = 3,
        WAIT_FEEDBACK = 4, FEEDBACK_TIMEOUT = 5, WAIT_NEUTRAL = 6,
        INVALID_RC = 7, INVALID_CONFIG = 8, OVER_TEMPERATURE = 9,
        INVALID_PID = 10, WRONG_MOTOR_MODE = 11
    };
    int32_t max_speed = 300; // 摇杆推满时目标绝对值；Resolve另有3800 RPM硬上限。
    int16_t deadband = 20; // 输入±20内目标为0，也是进入/恢复FIRE的回中条件。
    int8_t direction = -1; // 已确认：上推负转速送料，下推正转速退弹；不改反馈符号。
    uint32_t rc_timeout_ms = 100; // 新遥控帧间隔达到此值即停机。
    uint32_t feedback_timeout_ms = 100; // CAN2 ID5反馈间隔达到此值即停机。

    bool armed = false; // 是否已在双DOWN下收到回中帧；不等于当前一定允许运行。
    bool enabled = false; // Resolve综合模式、输入和反馈有效性后的使能结果。
    bool rc_seen = false;
    bool feedback_seen = false;
    int32_t target_speed = 0;
    uint32_t last_rc_ms = 0;
    uint32_t last_feedback_ms = 0;
    StopReason stop_reason = WAIT_RC;
    StopReason last_fault = RUNNING; // 等待回中期间保留最近一次异常原因，便于Live Watch追踪。

    void Disarm(StopReason reason = WAIT_NEUTRAL)
    {
        armed = false;
        enabled = false;
        target_speed = 0;
        stop_reason = reason;
        if (reason == RC_TIMEOUT || reason == FEEDBACK_TIMEOUT ||
            reason == INVALID_RC || reason == INVALID_CONFIG ||
            reason == OVER_TEMPERATURE || reason == INVALID_PID ||
            reason == WRONG_MOTOR_MODE)
            last_fault = reason;
    }

    // 仅实际收到的新RC帧调用；不能反复使用旧帧更新时间，否则失联保护失效。
    void OnRcFrame(uint8_t left, uint8_t right, int16_t axis, uint32_t tick)
    {
        if (left < 1 || left > 3 || right < 1 || right > 3 ||
            axis < -660 || axis > 660)
        {
            rc_seen = false;
            Disarm(INVALID_RC);
            return;
        }

        const bool fresh = rc_seen &&
            static_cast<uint32_t>(tick - last_rc_ms) < rc_timeout_ms;
        const bool next_fire = (left == 2 && right == 2);
        if (!fresh)
            Disarm(rc_seen ? RC_TIMEOUT : WAIT_NEUTRAL);
        else if (!double_down || !next_fire)
            Disarm();

        double_down = next_fire;
        stick = axis;
        last_rc_ms = tick;
        rc_seen = true;

        // 进入或异常恢复必须先回中；一直推住摇杆不会自行重新启动。
        if (double_down && axis >= -deadband && axis <= deadband)
            armed = true;
    }

    // CAN2 ID5有效反馈中断调用；即使任务错过了超时窗口，也能在下一帧发现间隔异常。
    void OnFeedback(uint32_t tick)
    {
        if (feedback_seen &&
            static_cast<uint32_t>(tick - last_feedback_ms) >= feedback_timeout_ms)
            Disarm(FEEDBACK_TIMEOUT);
        last_feedback_ms = tick;
        feedback_seen = true;
    }

    int32_t Resolve(bool fire_mode, uint32_t tick)
    {
        // 任一条件不满足都解除armed/enabled并将目标清零；不在此直接操作电机。
        // 检查顺序：模式→双DOWN→配置→RC时效→CAN反馈时效→回中解锁。
        StopReason reason = RUNNING;
        if (!fire_mode) reason = NOT_FIRE;
        else if (!rc_seen) reason = last_fault == INVALID_RC ? INVALID_RC : WAIT_RC;
        else if (!double_down) reason = NOT_FIRE;
        else if (deadband < 0 || deadband >= 660 || max_speed <= 0 ||
                 (direction != 1 && direction != -1))
            reason = INVALID_CONFIG;
        else if (static_cast<uint32_t>(tick - last_rc_ms) >= rc_timeout_ms)
            reason = RC_TIMEOUT;
        else if (!feedback_seen) reason = WAIT_FEEDBACK;
        else if (static_cast<uint32_t>(tick - last_feedback_ms) >= feedback_timeout_ms)
            reason = FEEDBACK_TIMEOUT;
        else if (!armed) reason = WAIT_NEUTRAL;

        if (reason != RUNNING)
        {
            Disarm(reason);
            return 0;
        }

        enabled = true;
        stop_reason = RUNNING;
        target_speed = 0;
        const int32_t magnitude = stick < 0 ? -static_cast<int32_t>(stick) : stick;
        if (magnitude <= deadband)
            return 0;

        const int32_t limit = max_speed > 3800 ? 3800 : max_speed;
        // 去除死区后线性映射并整数截断：上推+660→-300，回中→0，下推-660→+300。
        const int32_t rpm = (magnitude - deadband) * limit / (660 - deadband);
        target_speed = (stick < 0 ? -rpm : rpm) * direction;
        return target_speed;
    }

private:
    bool double_down = false;
    int16_t stick = 0;
};
