#include "feeder.h"
#include "motor.h"
#include "control.h"
#include "task.h"
#include <algorithm>
#include <cmath>

FEEDER feeder;

void FEEDER::Init(Motor& motor)
{
    // PID参数在STM32F405.cpp的supply电机初始化中设置；这里只清历史，不覆盖参数。
    Stop(motor);
    initialized = true;
}

void FEEDER::UpdateCommand(uint8_t left, uint8_t right, int16_t axis, uint32_t tick)
{
    // RC解码任务只发布新输入；临界区与电机任务/CAN反馈对command的访问保持一致。
    taskENTER_CRITICAL();
    command.OnRcFrame(left, right, axis, tick);
    taskEXIT_CRITICAL();
}

void FEEDER::OnFeedback(uint32_t tick)
{
    // CAN中断只记录反馈时间和超时恢复状态，不运行PID；这里不能调用任务版临界区API。
    command.OnFeedback(tick);
}

void FEEDER::Stop(Motor& motor)
{
    // 退出FIRE或保护触发：撤销驱动力并清PID历史；这不是机械锁止或整车急停。
    pid_output = 0.0f;
    speed_error = 0;
    motor.setspeed = 0;
    motor.setcurrent = 0;
    motor.current = 0;
    motor.spinning = false;
    motor.pd = false;
    motor.pid[speed].Reset();
}

void FEEDER::Update(Motor& motor)
{
    if (!initialized)
        Init(motor);

    taskENTER_CRITICAL();
    // 先屏蔽反馈中断再取时间，避免新反馈时间比tick更晚导致无符号减法下溢。
    const uint32_t tick = HAL_GetTick();
    rc_age_ms = command.rc_seen ? tick - command.last_rc_ms : UINT32_MAX;
    feedback_age_ms = command.feedback_seen ? tick - command.last_feedback_ms : UINT32_MAX;
    effective_current_limit = CurrentLimit(motor.maxcurrent);
    const int32_t target = command.Resolve(ctrl.mode == CONTROL::FIRE, tick);
    // 模块内保护不改变其他电机：过温、模式不匹配或限幅无效都要求停机后回中恢复。
    if (motor.temperature > 70)
        command.Disarm(FeederCommand::OVER_TEMPERATURE);
    else if (motor.mode != SPD)
        command.Disarm(FeederCommand::WRONG_MOTOR_MODE);
    else if (effective_current_limit <= 0)
        command.Disarm(FeederCommand::INVALID_CONFIG);
    const bool enabled = command.enabled;
    stop_reason = command.stop_reason;
    taskEXIT_CRITICAL();

    if (!enabled)
    {
        Stop(motor);
        return;
    }

    motor.setspeed = target;
    motor.spinning = (target != 0);
    motor.pd = false;

    // FIRE内回中仍执行零速闭环，电流未必为0；直接正反换向不清积分、不加速度斜坡。
    // Position第二参数30000限制的是累计误差，不是电流输出；I=0.1时积分项最大±3000。
    // 本实现按每次调用累计误差，没有显式dt；调任务周期后不能照搬原来的I/D参数。
    speed_error = motor.setspeed - motor.curspeed;
    const float output = motor.pid[speed].Position(
        static_cast<float>(speed_error), 30000.0f);
    pid_output = output;
    if (!std::isfinite(output))
    {
        taskENTER_CRITICAL();
        command.Disarm(FeederCommand::INVALID_PID);
        stop_reason = command.stop_reason;
        taskEXIT_CRITICAL();
        Stop(motor);
        return;
    }
    const int32_t limit = effective_current_limit;
    const float limited = std::max(-static_cast<float>(limit),
        std::min(static_cast<float>(limit), output));
    motor.setcurrent = static_cast<int32_t>(limited);
    // Motor::Ontimer随后再次按motor.maxcurrent限幅写current，再打包进CAN发送缓冲。
}
