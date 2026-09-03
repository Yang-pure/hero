#pragma once
#include <cinttypes>
#include <cstring>
#include <cmath>
#include "PID.h"
#include "kalman.h"
#include "label.h"
#define WHEEL_RADIUS_MM 47.5f
#define GEAR_RATIO      36.0f

constexpr auto MAXSPEED = 5000;
//constexpr auto ADJUSTSPEED = 3000;

enum { ID1 = 0x201, ID2, ID3, ID4, ID5, ID6, ID7, ID8 };
enum { pre = 0, now };
enum pid_mode { speed = 0, position, speed2 };
enum motor_type { M3508, M3510, M2310, EC60, M6623, M6020, M2006 };
enum motor_mode { SPD, POS, ACE ,wheel_left,wheel_right};
enum function_type { chassis, pantile, shooter, supply };
typedef enum { UNINIT, UNCONNECTED, DISCONNECTED, FINE }motor_status_t;
#define SQRTF(x) ((x)>0?sqrtf(x):-sqrtf(-x))
#define T 1.e-3f

class Motor
{
	typedef motor_type type_t;
public:
	uint32_t ID;
	Kalman kalman{ 1.f,40.f };
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _speed2);
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position);//ACE模式
	Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed);
	void recorded_the_Laps();
	void Ontimer(uint8_t idata[][8], uint8_t* odata);
private:
	motor_status_t m_status = UNINIT;
	int32_t old_torque_current = 0;
	int32_t disconnectCount = 0;
	const int32_t disconnectMax = 20;
	void getmax(const type_t type);
	void StatusIdentifier(int32_t torque_current);
	void GetDistanceFromMechanicalAngle();
	static int16_t getword(const uint8_t high, const uint8_t low);
	static int32_t setrange(const int32_t original, const int32_t range);
	type_t type;
public:
	function_type function;
	uint16_t need_curcircle;
	static int16_t getdeltaa(int16_t diff);
	uint8_t getStatus()const;
	int32_t current{}, curspeed{}, setcurrent{}, setspeed{}, torque_current, motor_status, motor_angle_status, sum_angle{}, testspeed{}, zero_angle{}, target_angle{}, position_error{};//这个current用于输出电流或者电压
	float speed_feedforward{};
	int16_t adjspeed{};
	int16_t maxspeed{}, maxcurrent{};
	Kalman currentKalman{ 1.f,40.f };
	int temperature;
	int32_t stopAngle;
	int32_t mode{};
	int round_count;
	bool pd = 0, spinning = 0, position_initialized{};//pd:单次拨弹 spinning:一秒八发

	// 多圈位置控制，仅给 Pitch 启用
	bool continuous_position = false;
	bool continuous_initialized = false;

	PID pid[2];
	float Torque_constant_2006 = (0.18*10)/10000;
	float setangle{}, angle[2]{},distance{}, initial_x{}, rota_angle{}, reset_rota_angle{}, delta_angle{};
	float Torque_left;
	float Torque_right;
	float const_dx = 0.004974;// m / s  /rpm
};
//此处要根据实际不同can线上的电机数量进行更改
extern Motor can1_motor[CAN1_MOTOR_NUM];
extern Motor can2_motor[CAN2_MOTOR_NUM];
