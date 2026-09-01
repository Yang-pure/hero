#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"

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
}


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	ch_pitch *= (-1.f);
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
		delta = degreeToMechanical(yaw_error_degree);

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
		if (ctrl.mode == CONTROL::ROTATION)
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
	if (ctrl.mode == RESET)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}
	
	Mecanum_Resolve(speedx, speedy, speedz);

}

void CONTROL::PANTILE::Update()
{
	if (ctrl.mode == RESET)
	{
		mark_yaw = para.initial_yaw;
		mark_pitch = para.initial_pitch;

		// 下次离开RESET时重新记录当前IMU角度
		yaw_hold_initialized = false;
	}

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
	if (mark_yaw < 0.0)mark_yaw += 8192.0;

	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

	ctrl.pantile_motor[PANTILE::YAW]->setangle = mark_yaw;
	//ctrl.pantile_motor[PANTILE::PITCH]->setangle = mark_pitch;
}

void CONTROL::SHOOTER::Update()
{
	//now_bullet_speed = judgement.data.ext_shoot_data_t.bullet_speed;
	if (ctrl.mode == RESET)
	{
		openRub = false;
		supply_bullet = false;
		auto_shoot = false;
	}
	if (openRub)
	{

	}
	else
	{

	}

	if (supply_bullet && openRub)
	{
		if (auto_shoot)
		{
			ctrl.supply_motor[0]->setspeed = 2160;
			ctrl.supply_motor[0]->spinning = true;
		}
		else
		{
			ctrl.supply_motor[0]->setspeed = 2160;
			ctrl.supply_motor[0]->spinning = true;
		}
	}
	else 
	{
		ctrl.supply_motor[0]->spinning = false;
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
