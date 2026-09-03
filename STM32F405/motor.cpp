#include "motor.h"
#include "gpio.h"
#include "HTmotor.h"
#include "imu.h"
#include "RC.h"
#include "feeder.h"
#define DEG_TO_RAD 0.017453292f  // π / 180
Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _speed2)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[speed2], &_speed2, sizeof(PID));
	this->function = function;
}


Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	this->function = function;
}

void Motor::StatusIdentifier(int32_t torque_current)
{
	if (torque_current == old_torque_current)
		disconnectCount++;
	else
		disconnectCount = 0;

	if (disconnectCount >= disconnectMax)
	{
		disconnectCount = disconnectMax;
		if (old_torque_current == 0)
			m_status = UNCONNECTED;
		else
			m_status = DISCONNECTED;
	}
	else
		m_status = FINE;

	old_torque_current = torque_current;
}
uint8_t Motor::getStatus()const
{
	return (uint8_t)m_status;
}
void Motor::Ontimer(uint8_t idata[][8], uint8_t* odata)//idate: receive;odate: trainsmit;RC
{
	uint32_t trainsmit_or_receive_ID = this->ID - ID1;

	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID += 4;
	}*/
	//----------------------------------------------------------------
	this->torque_current = getword(idata[trainsmit_or_receive_ID][4], idata[trainsmit_or_receive_ID][5]);
	this->StatusIdentifier(this->torque_current);
	this->angle[now] = getword(idata[trainsmit_or_receive_ID][0], idata[trainsmit_or_receive_ID][1]);
	this->temperature = idata[trainsmit_or_receive_ID][6];
	//Get currrent speed

	motor_status = 0;
	if (temperature > 70) {
		setspeed = 0;
	}

	if (type == EC60)
	{
		curspeed = static_cast<float>(getdeltaa(angle[now] - angle[pre])) / T / 8192.f * 60.f;
	}
	else {
		curspeed = getword(idata[trainsmit_or_receive_ID][2], idata[trainsmit_or_receive_ID][3]);
	}
	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID -= 4;
	}*/
	//----------------------------------------------------------------
	//20220121--hz
	recorded_the_Laps();
	if (mode == ACE)
	{

		if (spinning)
		{

		}
		else {
			if (need_curcircle > 0)
			{
				

			}
			else if (need_curcircle <= 0)
			{

			}
		}
		if (setspeed == 0 && curspeed == 0)
		{
			motor_status = 1;
			motor_angle_status = angle[0];
		}
		if (motor_status == 1 && fabs(motor_angle_status - angle[0]) < 50)
		{
			current = 0;
		}
	}
	else if (mode == POS)
	{
		// 位置外环：角度误差 → 目标速度
		position_error = getdeltaa(setangle - angle[now]);

		setspeed =
			pid[position].Position(position_error, 10000)
			+ speed_feedforward;

		setspeed = setrange(setspeed, 1500);

		// 速度内环：速度误差 → 目标电流
		setcurrent =
			pid[speed].Position(setspeed - curspeed, 10000);

		// 只对Yaw GM6020增加摩擦补偿
		if (type == M6020 && function == pantile)
		{
			bool in_jam_zone =
				angle[now] >= 6000.0f
				|| angle[now] <= 1600.0f;

			constexpr int32_t yaw_motion_current_ff = 4000;
			constexpr int32_t jam_current_ff = 1500;

			if (setspeed > 1)
			{
				// 整圈基础静摩擦补偿
				setcurrent += yaw_motion_current_ff+350;

				// 固定高阻力区额外补偿
				if (in_jam_zone)
				{
					setcurrent += jam_current_ff;
				}
			}
			else if (setspeed < -3)
			{
				setcurrent -= yaw_motion_current_ff;

				if (in_jam_zone)
				{
					setcurrent -= jam_current_ff;
				}
			}

			setcurrent = setrange(setcurrent, 8000);
		}
	}

	else if (mode == SPD)
	{
		if (function == supply)
			// ID5供弹轮由FEEDER统一完成使能、速度PID、限流和停止保护。
			feeder.Update(*this);
		else
			setcurrent = pid[speed].Position(setspeed - curspeed, 10000);
		////   双MID立即清零，作为恒流测试的紧急停止
		//if (rc.rc.s[0] == RC::MID
		//	&& rc.rc.s[1] == RC::MID)
		//{
		//	setcurrent = 0;
		//}
		//else
		//{
		//	// 临时将testspeed作为恒定电流指令
		//	setcurrent = testspeed;
		//}
	}

	/*
 * Yaw 6020固定机械阻力区补偿。
 * 当前卡涩区跨越编码器零点：
 * [6000, 8191] ∪ [0, 1600]
 */
	//if (type == M6020 && function == pantile)
	//{
	//	bool in_jam_zone =
	//		angle[now] >= 6000.0f
	//		|| angle[now] <= 1600.0f;

	//	if (in_jam_zone)
	//	{
	//		constexpr int32_t jam_current_ff = 1500;

	//		if (setspeed > 3)
	//		{
	//			setcurrent += jam_current_ff;
	//		}
	//		else if (setspeed < -3)
	//		{
	//			setcurrent -= jam_current_ff;
	//		}
	//	}

	//	// Yaw调试期间的独立安全限流
	//	setcurrent = setrange(setcurrent, 8000);
	//}
	
	GetDistanceFromMechanicalAngle();
	angle[pre] = angle[now];
	current = setrange(setcurrent, maxcurrent);
	odata[trainsmit_or_receive_ID * 2] = (current & 0xff00) >> 8;//高八位
	odata[trainsmit_or_receive_ID * 2 + 1] = current & 0x00ff;
}
void Motor::recorded_the_Laps() {
	int16_t delta = angle[now] - angle[pre];
	// 处理回绕：顺时针
	if (delta > 8192 / 2)
		delta -= 8192;
	// 处理回绕：逆时针
	else if (delta < -8192 / 2)
		delta += 8192;

	sum_angle+= delta;
//	round_count = total_count / encoder_resolution;
}

uint8_t initial_cnt=0;
void Motor::GetDistanceFromMechanicalAngle() {
	if (initial_cnt<5)
	initial_cnt++;
	distance=(6.2831853f/ 8192.0f)*sum_angle * (WHEEL_RADIUS_MM / GEAR_RATIO)-initial_x;  // 单位：mm

	if(initial_cnt<3)
	initial_x = distance;
}

void Motor::getmax(const type_t type)
{
	adjspeed = 3000;
	switch (type)
	{
	case M3508:
		maxcurrent = 16384;
		maxspeed = 3800;
		break;
	case M3510:
		maxcurrent = 13000;
		maxspeed = 9000;
		break;
	case M2310:
		maxcurrent = 13000;
		maxspeed = 9000;
		adjspeed = 1000;
		break;
	case EC60:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6623:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6020:
		maxcurrent = 30000;
		maxspeed = 200;
		adjspeed = 80;
		break;
	case M2006:
		maxcurrent = 10000;
		adjspeed = 1000;
		maxspeed = 3000;
		break;
	default:;
	}
}

int16_t Motor::getdeltaa(int16_t diff)
{
	if (diff <= -4096)
		diff += 8192;
	else if (diff > 4096)
		diff -= 8192;
	return diff;
}

int16_t Motor::getword(const uint8_t high, const uint8_t low)
{
	const int16_t word = high;
	return (word << 8) + low;
}

int32_t Motor::setrange(const int32_t original, const int32_t range)
{
	return std::max(std::min(range, original), -range);
}

