#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "RC.h"
#include "gpio.h"

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;  
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			supply_motor[num4] = motor[i];
			supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			num4++;
			break;
		default:
			break;
		}
	}
	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	pantile.mark_yaw = para.initial_yaw;
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;

	// 只给 Pitch 启用连续多圈位置
	pantile_motor[PANTILE::TYPE::PITCH]->continuous_position = true;
}


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	ch_pitch *= (-10.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float adjangle = this->pantile.sensitivity * 2;

	ctrl.pantile.mark_pitch -= (float)(adjangle * ch_pitch);
	ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep,PANTILE::TYPE type,IMU& frameOfReference)
{
	float delta = 0.0f;

	if (type == YAW)
	{
		/*
		 * 世界坐标系Yaw误差，单位：度。
		 */
		float yaw_error_degree =ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw());

		/*
		 * 转成GM6020编码值。
		 */
		delta = -degreeToMechanical(yaw_error_degree);

		if (delta <= -4096.0f)
		{
			delta += 8192.0f;
		}
		else if (delta >= 4096.0f)
		{
			delta -= 8192.0f;
		}

		/*
		 * 小陀螺模式下必须每周期刷新机械目标。
		 *
		 * 这样Motor内部的position_error始终近似等于
		 * 当前世界Yaw误差，而不会因为mark_yaw冻结，
		 * 变成旧机械角度误差。
		 */
		if (ctrl.mode == CONTROL::ROTATION || ctrl.mode == CONTROL::MANUAL_YAW || ctrl.mode == CONTROL::FOLLOW)
		{
			mark_yaw =ctrl.pantile_motor[YAW]->angle[now] + delta;
		}
		else
		{
			/*
			 * 非小陀螺模式可以保留原死区。
			 * 进入死区时把目标设为当前机械角，
			 * 避免保留旧目标继续施力。
			 */
			if (fabs(delta) >= 30.0f)
			{
				mark_yaw = ctrl.pantile_motor[YAW]->angle[now] + delta;
			}
			else
			{
				mark_yaw = ctrl.pantile_motor[YAW]->angle[now];
			}
		}
	}
	else if (type == PITCH)
	{
		/*
		 * 你的原PITCH代码保持不变。
		 */
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch()));

		if (delta <= -4096.0f)
		{
			delta += 8192.0f;
		}
		else if (delta >= 4096.0f)
		{
			delta -= 8192.0f;
		}

		if (fabs(delta) < 50.0f)
		{
			keep_PID[PITCH].m_error[INTEGRATE] = 0;
		}

		if (fabs(delta) >= 10.0f)
		{
			mark_pitch = base_mark_pitch	+ keep_PID[PITCH].Position(delta, 10000);
		}
	}
}

void CONTROL::CHASSIS::Keep_Direction()
{


}

void CONTROL::CHASSIS::Mecanum_Resolve(
	int32_t vx,
	int32_t vy,
	int32_t wz)
{
	int32_t wheel_speed[4] =
	{
		vx + vy - wz,  // 0 左前
		vx - vy + wz,  // 1 右前
		vx + vy + wz,  // 2 右后
		vx - vy - wz   // 3 左后
	};

	int32_t max_abs_speed = 1;

	for (int i = 0; i < 4; i++)
	{
		int32_t abs_speed;

		if (wheel_speed[i] >= 0)
			abs_speed = wheel_speed[i];
		else
			abs_speed = -wheel_speed[i];

		if (abs_speed > max_abs_speed)
			max_abs_speed = abs_speed;
	}

	float scale = 1.0f;

	if (max_abs_speed > para.max_speed)
	{
		scale = static_cast<float>(para.max_speed) / max_abs_speed;
	}

	/*
	 * 轮子位置：
	 * 左前 ID5 = can1_motor[0]
	 * 右前 ID6 = can1_motor[1]
	 * 左后 ID8 = can1_motor[3]
	 * 右后 ID7 = can1_motor[2]
	 *
	 * 方向不对时，只改对应的 1 为 -1。
	 */
	const int8_t motor_direction[4] =
	 {
		 1,   // 0 左前
		 -1,  // 1 右前
		 -1,  // 2 右后
		 1    // 3 左后
	 };

	ctrl.chassis_motor[0]->setspeed = wheel_speed[0] * scale * motor_direction[0];
	ctrl.chassis_motor[1]->setspeed = wheel_speed[1] * scale * motor_direction[1];
	ctrl.chassis_motor[2]->setspeed = wheel_speed[2] * scale * motor_direction[2];
	ctrl.chassis_motor[3]->setspeed = wheel_speed[3] * scale * motor_direction[3];
}

void CONTROL::CHASSIS::Update()
{
	// RESET和SHOOT模式下，底盘四轮目标速度强制归零
	if (ctrl.mode == RESET || ctrl.mode == SHOOT)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}

	Mecanum_Resolve(speedx, speedy, speedz);
}

void CONTROL::PANTILE::Update()
{
	Motor* pitch =
		ctrl.pantile_motor[PANTILE::PITCH];

	/*
	 * Pitch 使用连续编码器位置。
	 * 初始化完成后，以当前 sum_angle 作为本次上电的 home。
	 */
	if (pitch->continuous_position
		&& pitch->continuous_initialized
		&& !pitch_position_initialized)
	{
		pitch_home = (float)pitch->sum_angle;
		base_mark_pitch = pitch_home;
		mark_pitch = pitch_home;
		pitch_position_initialized = true;
	}

	if (ctrl.mode == RESET)
	{
		// Yaw 的原 RESET 逻辑保持不变
		mark_yaw = para.initial_yaw;

		// Pitch 回到本次上电记录的 home
		if (pitch_position_initialized)
		{
			mark_pitch = pitch_home;
		}

		yaw_hold_initialized = false;
	}

	// Yaw 继续使用原来的 0～8192 单圈处理
	if (mark_yaw > 8192.0f)
	{
		mark_yaw -= 8192.0f;
	}

	if (mark_yaw < 0.0f)
	{
		mark_yaw += 8192.0f;
	}

	ctrl.pantile_motor[PANTILE::YAW]->setangle =
		mark_yaw;

	// Pitch 使用相对于上电 home 的连续位置限位
	if (pitch_position_initialized)
	{
		const float pitch_low =
			pitch_home + para.pitch_min;

		const float pitch_high =
			pitch_home + para.pitch_max;

		mark_pitch =
			std::max(
				std::min(mark_pitch, pitch_high),
				pitch_low
			);

		pitch->setangle = mark_pitch;
	}
}

void CONTROL::SHOOTER::Update()
{
	/*
	 * 使用功能指针，不直接使用参考工程的can2_motor数组下标。
	 *
	 * 参考工程：
	 * can2_motor[1..3] = 摩擦轮
	 *
	 * 当前主工程：
	 * can2_motor[0..2] = 摩擦轮
	 * can2_motor[3]    = Yaw
	 * can2_motor[4]    = 拨弹轮
	 * can2_motor[5]    = Pitch
	 *
	 * 使用shooter_motor和supply_motor后，CAN2数组顺序变化不会影响发射逻辑。
	 */
	Motor* friction_motor_1 = ctrl.shooter_motor[0];
	Motor* friction_motor_2 = ctrl.shooter_motor[1];
	Motor* friction_motor_3 = ctrl.shooter_motor[2];
	Motor* feeder_motor = ctrl.supply_motor[0];

	/*
	 * 只允许SHOOT模式驱动发射机构。
	 *
	 * 参考工程只在RESET时清零，但当前主工程还有ROTATION、
	 * FOLLOW、MANUAL_YAW等模式。如果从SHOOT直接切换到这些模式，
	 * 必须清除上一周期目标，防止摩擦轮或拨弹轮继续运转。
	 */
	if (ctrl.mode != CONTROL::SHOOT)
	{
		supply_bullet = false;
		push_state = WAIT;
		push_timer = 0;

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

		direction_trigger = 0.0f;

		friction_motor_1->setspeed = 0.0f;
		friction_motor_2->setspeed = 0.0f;
		friction_motor_3->setspeed = 0.0f;
		feeder_motor->setspeed = 0.0f;

		last_single_trigger = false;
		single_latch = false;

		return;
	}

	/*
	 * 进入SHOOT模式后，三摩擦轮持续转动。
	 * 转向和速度完全采用参考发射工程。
	 */
	friction_motor_1->setspeed = -2000.0f;
	friction_motor_2->setspeed = 2000.0f;
	friction_motor_3->setspeed = -2000.0f;

	/*
	 * ch[3]：连发
	 * ch[2]：单发
	 *
	 * 摇杆正负方向同时决定拨弹轮正反转。
	 */
	const int16_t ch_burst = rc.rc.ch[3];
	const int16_t ch_single = rc.rc.ch[2];

	bool burst_trigger = false;
	bool single_trigger = false;

	if (ch_burst >= 500 && ch_burst <= 660)
	{
		burst_trigger = true;
		direction_trigger = 1.0f;
	}
	else if (ch_burst <= -500 && ch_burst >= -660)
	{
		burst_trigger = true;
		direction_trigger = -1.0f;
	}

	if (ch_single >= 500 && ch_single <= 660)
	{
		single_trigger = true;
		direction_trigger = 1.0f;
	}
	else if (ch_single <= -500 && ch_single >= -660)
	{
		single_trigger = true;
		direction_trigger = -1.0f;
	}

	/*
	 * 单发采用上升沿触发。
	 * 摇杆一直停在触发位置时，不会反复产生新的单发命令。
	 */
	const bool single_rising =
		single_trigger && !last_single_trigger;

	last_single_trigger = single_trigger;

	if (single_rising)
	{
		single_latch = true;
	}

	const bool shoot_cmd =
		single_rising || burst_trigger;

	/*
	 * 单发锁存有效时，即使摇杆已经回中，
	 * 也要继续完成当前这一发。
	 */
	const bool firing =
		shoot_cmd || single_latch;

	if (firing)
	{
		supply_bullet = true;
	}
	else if (push_state == WAIT)
	{
		supply_bullet = false;
	}

	if (!supply_bullet)
	{
		feeder_motor->setspeed = 0.0f;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

		push_state = WAIT;
		push_timer = 0;
		return;
	}

	const bool bullet_detected =
		HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET;

	const uint32_t now_tick = HAL_GetTick();

	switch (push_state)
	{
	case WAIT:
		/*
		 * 微动开关尚未触发时，拨弹轮继续送料。
		 */
		feeder_motor->setspeed =
			300.0f * direction_trigger;

		/*
		 * PA8为高：弹丸已经到达推杆位置。
		 * 先停拨弹轮，再推出电推杆。
		 */
		if (bullet_detected && firing)
		{
			feeder_motor->setspeed = 0.0f;

			HAL_GPIO_WritePin(
				GPIOC,
				GPIO_PIN_9,
				GPIO_PIN_SET);

			push_state = PUSHING;
			push_timer = now_tick;
		}
		break;

	case PUSHING:
		/*
		 * 推杆推出期间禁止拨弹轮继续送料。
		 */
		feeder_motor->setspeed = 0.0f;

		if (now_tick - push_timer >= PUSH_HOLD_TIME)
		{
			HAL_GPIO_WritePin(
				GPIOC,
				GPIO_PIN_9,
				GPIO_PIN_RESET);

			push_state = BACK;
			push_timer = now_tick;
		}
		break;

	case BACK:
		/*
		 * PC9已经拉低，等待推杆机械回位。
		 */
		feeder_motor->setspeed = 0.0f;

		if (now_tick - push_timer >= RETRACT_WAIT_TIME)
		{
			push_state = WAIT;
			push_timer = 0;

			/*
			 * 连发保持时，回到WAIT后继续送料。
			 */
			feeder_motor->setspeed =
				300.0f * direction_trigger;

			/*
			 * 不是连发，就表示当前单发已经完成。
			 */
			if (!burst_trigger)
			{
				single_latch = false;
			}

			/*
			 * 当前既没有新的单发沿，也没有连发命令，
			 * 完成这一发后停止送料。
			 */
			if (!shoot_cmd)
			{
				supply_bullet = false;
			}
		}
		break;

	default:
		feeder_motor->setspeed = 0.0f;

		HAL_GPIO_WritePin(
			GPIOC,
			GPIO_PIN_9,
			GPIO_PIN_RESET);

		push_state = WAIT;
		push_timer = 0;
		supply_bullet = false;
		break;
	}
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	// 归一化到 -180 ~ 180，处理角度回绕
	while (delta > 180.f) delta -= 360.f;
	while (delta <= -180.f) delta += 360.f;
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}




extern uint8_t Power_stsRx[];
