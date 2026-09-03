#include "label.h"
#include "RC.h"
#include "control.h"


void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 或者报错
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	}

	if (pd_Rx != pdTRUE) return; // no stale frame counted as a new command
	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C 
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E

	
}

void RC::OnRC()
{
	/*
	 * 双DOWN进入完整发射模式。
	 *
	 * 参考工程原来使用UP+UP，但你的主工程已经将UP+UP
	 * 用作ROTATION。为了保留你的底盘和Yaw模式，
	 * 使用旧FIRE占用的双DOWN位置进入SHOOT。
	 */
	if (rc.s[0] == DOWN && rc.s[1] == DOWN)
	{
		ctrl.mode = CONTROL::SHOOT;

		// 发射时锁死底盘三个运动目标
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;

		/*
		 * 直接返回，避免ch[2]、ch[3]继续被底盘逻辑使用。
		 * SHOOTER::Update()会把它们解释为单发和连发通道。
		 */
		return;
	}

	/*
	 * 从SHOOT切换到一个暂时没有定义功能的拨杆组合时，
	 * 不能继续残留在SHOOT模式。
	 */
	if (ctrl.mode == CONTROL::SHOOT)
	{
		ctrl.mode = CONTROL::RESET;
	}

	// 以下模式映射保持你的主工程逻辑
	if (rc.s[0] == MID && rc.s[1] == MID)
	{
		ctrl.mode = CONTROL::RESET;
	}
	else if (rc.s[0] == UP && rc.s[1] == MID)
	{
		ctrl.mode = CONTROL::FOLLOW;
	}
	else if (rc.s[0] == UP && rc.s[1] == UP)
	{
		ctrl.mode = CONTROL::ROTATION;
	}
	else if (rc.s[0] == MID && rc.s[1] == UP)
	{
		ctrl.mode = CONTROL::MANUAL_YAW;
	}
	else if (rc.s[0] == DOWN && rc.s[1] == UP)
	{
		// 暂未定义，保持你原来的空逻辑
	}
	else if (rc.s[0] == DOWN && rc.s[1] == MID)
	{
		// 暂未定义，保持你原来的空逻辑
	}
	else if (rc.s[0] == MID && rc.s[1] == DOWN)
	{
		// 暂未定义，保持你原来的空逻辑
	}
	else if (rc.s[0] == UP && rc.s[1] == DOWN)
	{
		// 暂未定义，保持你原来的空逻辑
	}

	if (Shift_mode())
	{
		// 保持你原来的空逻辑
	}

	if (ctrl.mode == CONTROL::RESET)
	{
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;
	}

	if (ctrl.mode != CONTROL::RESET)
	{
		ctrl.chassis.speedx =
			rc.ch[1] * para.max_speed / 660.0f;

		ctrl.chassis.speedy =
			rc.ch[0] * para.max_speed / 660.0f;

		if (ctrl.mode == CONTROL::ROTATION)
		{
			ctrl.chassis.speedz = 1000;
			ctrl.chassis.Keep_Direction();
		}
		else if (ctrl.mode == CONTROL::FOLLOW)
		{
			ctrl.chassis.speedz =
				rc.ch[2] * para.rota_speed / 660.0f;
		}
		else if (ctrl.mode == CONTROL::MANUAL_YAW)
		{
			ctrl.chassis.speedz = 0;
		}
	}
}

void RC::OnPC()
{
	;
}

void RC::Update()
{
	OnRC();
	OnPC();
}




void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}
