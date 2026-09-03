#pragma once
#include "feeder_command.h"
#include <algorithm>

class Motor;

// 单个手动供弹轮：CAN2 ID5 / can2_motor[4]，只在双DOWN的FIRE模式运行。
// 输入/反馈入口只更新command；电机更新任务统一计算PID和写电流，避免多处抢写。
// 调参入口：本文件改电流，feeder_command.h改速度/方向，STM32F405.cpp改PID。
class FEEDER
{
public:
    FeederCommand command; // 摇杆映射参数、输入时效、回中解锁和停止原因
    int32_t max_current = 30000; // 请求电流指令上限（不是mA），不强制PID输出此值。
    int32_t effective_current_limit = 0; // 只读观察量：请求值、motor.maxcurrent、16384取最小。
    float pid_output = 0.0f; // 只读观察量：限幅前PID输出；禁止运行时清零。
    int32_t speed_error = 0; // 本周期setspeed - curspeed，单位为电机反馈RPM。
    uint32_t rc_age_ms = UINT32_MAX;
    uint32_t feedback_age_ms = UINT32_MAX;
    FeederCommand::StopReason stop_reason = FeederCommand::WAIT_RC;

    int32_t CurrentLimit(int32_t motor_limit) const
    {
        // C620发送范围为±16384；不能照搬GM6020的±30000，也不改其他M3508的公共限幅。
        return std::max<int32_t>(0, std::min<int32_t>(16384,
            std::min<int32_t>(max_current, motor_limit)));
    }

    void Init(Motor& motor); // 首次电机更新时清输出和PID历史，不重设PID参数。
    void UpdateCommand(uint8_t left, uint8_t right, int16_t axis, uint32_t tick); // 仅新RC帧调用。
    void OnFeedback(uint32_t tick); // 仅CAN2 ID5有效反馈中断调用。
    void Update(Motor& motor); // 检查使能→目标速度→PID→电流限幅。
    void Stop(Motor& motor); // 电机任务内部清零助手；外部锁存停机还必须解除command使能。

private:
    bool initialized = false;
};

extern FEEDER feeder;
