#include "motor.h"
#include "gpio.h"
#include "HTmotor.h"
#include "imu.h"
#include "RC.h"

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

	if (continuous_position && !continuous_initialized)
	{
		// 第一次初始化时，以当前机械角作为连续位置起点
		angle[pre] = angle[now];
		sum_angle = angle[now];
		setangle = (float)sum_angle;
		continuous_initialized = true;
	}

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
		// Pitch 使用连续多圈位置；Yaw 使用单圈最短路径位置误差
		if (continuous_position)
		{
			position_error =
				static_cast<int32_t>(setangle - static_cast<float>(sum_angle));
		}
		else
		{
			position_error = getdeltaa(setangle - angle[now]);
		}

		// 位置外环：位置误差 -> 目标速度
		setspeed =
			pid[position].Position(position_error, 10000) +
			speed_feedforward;

		// Pitch 保持原来的500；
		// Yaw初次调位置环时限制在±50，避免突然高速运动
		setspeed = setrange(
			setspeed,
			continuous_position ? 500 : 50
		);

		// 速度内环：目标速度与实际速度误差 -> 电流
		setcurrent =
			pid[speed].Position(setspeed - curspeed, 10000);

		// Yaw GM6020分区摩擦补偿
		if (type == M6020 && function == pantile)
		{
			/*
			 * 这些补偿是在约20rpm下标定的。
			 * 目标速度低于20rpm时按比例加入，避免低速时补偿过大。
			 */
			float yaw_ff_scale =
				fabs(static_cast<float>(setspeed)) / 20.0f;

			if (yaw_ff_scale > 1.0f)
			{
				yaw_ff_scale = 1.0f;
			}

			if (setspeed > 0)
			{
				setcurrent += static_cast<int32_t>(
					5000.0f * yaw_ff_scale
					);

				if (angle[now] >= 6000.0f)
				{
					setcurrent += static_cast<int32_t>(
						1500.0f * yaw_ff_scale
						);
				}
				else if (angle[now] <= 2200.0f)
				{
					setcurrent += static_cast<int32_t>(
						2500.0f * yaw_ff_scale
						);
				}
				else if (angle[now] >= 3600.0f &&
					angle[now] <= 5600.0f)
				{
					setcurrent += static_cast<int32_t>(
						500.0f * yaw_ff_scale
						);
				}
			}
			else if (setspeed < 0)
			{
				setcurrent -= static_cast<int32_t>(
					5000.0f * yaw_ff_scale
					);

				if (angle[now] >= 6000.0f)
				{
					setcurrent -= static_cast<int32_t>(
						1700.0f * yaw_ff_scale
						);
				}
				else if (angle[now] <= 800.0f)
				{
					setcurrent -= static_cast<int32_t>(
						2300.0f * yaw_ff_scale
						);
				}
				else if (angle[now] >= 3200.0f &&
					angle[now] <= 5600.0f)
				{
					setcurrent -= static_cast<int32_t>(
						1600.0f * yaw_ff_scale
						);
				}

				if (angle[now] >= 6800.0f &&
					angle[now] <= 8000.0f)
				{
					setcurrent -= static_cast<int32_t>(
						900.0f * yaw_ff_scale
						);
				}

				if (angle[now] >= 4000.0f &&
					angle[now] <= 5600.0f)
				{
					setcurrent -= static_cast<int32_t>(
						800.0f * yaw_ff_scale
						);
				}

				// 此区间原先反向速度偏快，适当削弱负向电流
				if (angle[now] >= 1200.0f &&
					angle[now] <= 2600.0f)
				{
					setcurrent += static_cast<int32_t>(
						1200.0f * yaw_ff_scale
						);
				}
			}

			setcurrent = setrange(setcurrent, 16000);
		}
	}

	else if (mode == SPD)
	{
		if (type == M6020 && function == pantile)
		{
			//// Yaw 恒定输出测试：
			//// testspeed 临时作为输出指令，不再表示目标转速。
			//// 绕过速度 PID，保留原来的 ±16000 输出限幅。
			//setcurrent = testspeed;
			// Yaw速度环测试：testspeed表示目标转速。
			setspeed = testspeed;

			// PID根据目标速度与实际速度的差值，增减输出。
			setcurrent = pid[speed].Position(setspeed - curspeed, 10000);

			// 暂时保留5000作为正反向基准输出。
			// 目标为0时不加基准，PID仍可对残余转速产生制动输出。
			if (setspeed > 0)
				setcurrent += 5000;
			else if (setspeed < 0)
				setcurrent -= 5000;
			// 正方向补偿：跨零前保持1500，跨零后先提高到2000。
			// 正方向分区补偿，反方向不受影响。
			if (testspeed > 0)
			{
				// 跨零前。
				if (angle[now] >= 6000.0f)
					setcurrent += 1500;

				// 跨零后：2500暂时保留，不再提高。
				else if (angle[now] <= 2200.0f)
					setcurrent += 2500;

				// 另一侧低速区：显示角度9～14，先试补500。
				else if (angle[now] >= 3600.0f &&
					angle[now] <= 5600.0f)
					setcurrent += 500;
			}
			else if (testspeed < 0)
			{
				// 基础补偿。
				if (angle[now] >= 6000.0f)
					setcurrent -= 1700;
				else if (angle[now] <= 800.0f)
					setcurrent -= 2300;  // 0°附近：保持
				else if (angle[now] >= 3200.0f &&
					angle[now] <= 5600.0f)
					setcurrent -= 1600;

				// 60°附近：额外补偿600改为900。
				// 总补偿1700 + 900 = 2600。
				if (angle[now] >= 6800.0f && angle[now] <= 8000.0f)
					setcurrent -= 900;

				// 180°附近：额外补偿500改为800。
				// 总补偿1600 + 800 = 2400。
				if (angle[now] >= 4000.0f && angle[now] <= 5600.0f)
					setcurrent -= 800;

				// 约263°～325°：覆盖270°这一侧的较快区。
// 反向输出为负，加300表示减小驱动，不是加速。
				if (angle[now] >= 1200.0f && angle[now] <= 2600.0f)
					setcurrent += 1200;
			}

			setcurrent = setrange(setcurrent, 16000);
		}
		else
		{
			// 其他电机仍按原来的目标速度运行，不受 Yaw 测试影响。
			setcurrent = pid[speed].Position(setspeed - curspeed, 10000);
		}
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

