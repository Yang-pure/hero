#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"
#include"HTmotor.h"
#include "pid.h"

class CONTROL final
{
public:
	uint8_t init_DM = 0;
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};
	
	enum MODE { ROTATION, RESET, MANUAL_YAW, FOLLOW, LOCK, TEST, AUTO, SHOOT } ;

	MODE mode = RESET;
	struct CHASSIS
	{
		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};
		
		void Keep_Direction();
		void Mecanum_Resolve(int32_t vx, int32_t vy, int32_t wz);
		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{
		enum TYPE { YAW, PITCH };
		float mark_pitch{}, mark_yaw{};
		float base_mark_yaw{};      // YAW基准位置（进入保持模式时的电机角度）
		float base_mark_pitch{};    // PITCH基准位置

		// Pitch 连续位置基准
		float pitch_home{};
		bool pitch_position_initialized = false;
		// control.h
		PID pantile_PID[3] = { {0.5f, 0.01f, 0.f}, {0.5f, 0.01f, 0.f}, {0.f, 0.f, 0.f} };
		// 或
		PID keep_PID[2] = { {2.0f, 0.05f, 0.1f, 0.f}, {2.0f, 0.05f, 0.1f, 0.f} };
		float pid_pantile_out_speed{};

		const float sensitivity = 2.5f;
		bool aim = false;
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU& frameOfReference);
		void Update();

		float set_yaw{};                 // 世界目标YAW，单位：度
		bool yaw_hold_initialized = false;
		/*
 * Yaw速度前馈相关参数。
 *
 * yaw_cmd_ff_k：
 * 将底盘speedz指令转换成Yaw电机目标速度。
 *
 * yaw_gyro_fb_k：
 * 将IMU世界Yaw角速度转换成附加速度补偿。
 *
 * 两个系数的正负方向必须通过实车确认。
 */
		float yaw_cmd_ff_k = 0.015f;
		float yaw_gyro_fb_k = -0.0f;

		float yaw_speed_ff{};
		float yaw_speed_ff_target{};

		float yaw_ff_limit = 300.0f;
		float yaw_ff_filter = 0.25f;
	};

	struct SHOOTER
	{
		// 当前拨弹方向：1或-1
		float direction_trigger = 0.0f;

		float now_bullet_speed = 0.0f;

		// 单发通道上一周期状态，用于检测上升沿
		bool last_single_trigger = false;

		bool auto_shoot = false;
		bool openRub = false;
		bool supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;

		// 单发锁存：松开摇杆后，也要完成当前一次推弹流程
		bool single_latch = false;

		// 保留参考工程参数
		int16_t shoot_speed = 600;

		enum PushState
		{
			WAIT,       // 等待弹丸到达微动开关
			PUSHING,    // 电推杆正在推出
			BACK        // 电推杆已经释放，等待机械回位
		};

		PushState push_state = WAIT;

		// 状态切换时间，单位ms
		uint32_t push_timer = 0;

		// 电推杆高电平保持50ms
		static constexpr uint32_t PUSH_HOLD_TIME = 50;

		// 拉低后等待30ms再进行下一发
		static constexpr uint32_t RETRACT_WAIT_TIME = 30;

		void Update();
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;
	
	static int16_t Setrange(const int16_t original, const int16_t range);
	void Control_Pantile(int32_t ch_yaw, int32_t ch_pitch);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
	void init_dm();

private:

};

extern CONTROL ctrl;
