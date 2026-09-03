#include "label.h"
#include "RC.h"
#include "control.h"
#include "feeder.h"

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

	feeder.UpdateCommand(rc.s[0], rc.s[1], rc.ch[1], HAL_GetTick());
}

void RC::OnRC()
{
	if (UpdateFireMode()) return;

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

	}
	else if (rc.s[0] == DOWN && rc.s[1] == MID)
	{

	}

	else if (rc.s[0] == MID && rc.s[1] == DOWN)
	{

	}
	else if (rc.s[0] == UP && rc.s[1] == DOWN)
	{
		;
	}
	if (Shift_mode())
	{

	}
	if (ctrl.mode == CONTROL::RESET)
	{
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;
	}
	//pantile_yaw保持不动
	//ROTATION：：恒速转z
	//！ROTATION：：遥控定wz
	if (ctrl.mode != CONTROL::RESET)
	{

		ctrl.chassis.speedx = rc.ch[1] * para.max_speed / 660.f;
		ctrl.chassis.speedy = rc.ch[0] * para.max_speed / 660.f;
		//ctrl.chassis.speedz = rc.ch[2] * para.rota_speed / 660.0f;
		//ctrl.Control_Pantile(rc.ch[2] * para.yaw_speed / 660.f, rc.ch[3] * para.pitch_speed / 660.f);

		if (ctrl.mode == CONTROL::ROTATION)
		{
			ctrl.chassis.speedz = 1000;
			ctrl.chassis.Keep_Direction();
		}
		else if (ctrl.mode == CONTROL::FOLLOW)
		{
			ctrl.chassis.speedz = rc.ch[2] * para.rota_speed / 660.0f;
			
		}
		else if (ctrl.mode == CONTROL::MANUAL_YAW)
		{
			ctrl.chassis.speedz = 0;
		}
		/*ctrl.chassis.speedx = rc.ch[1] * para.max_speed / 660.0f;

		ctrl.chassis.speedy =rc.ch[0] * para.max_speed / 660.0f;

		ctrl.chassis.speedz =rc.ch[2] * para.rota_speed / 660.0f;*/

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

bool RC::UpdateFireMode()
{
	// 双DOWN是纯供弹模式：底盘目标清零，右摇杆竖直轴交给FEEDER，返回后不再执行旧底盘映射。
	const bool fire_selected = rc.s[0] == DOWN && rc.s[1] == DOWN;
	if (!fire_selected)
	{
		// 离开双DOWN后不能残留FIRE；FEEDER随后会因非FIRE而清零输出。
		if (ctrl.mode == CONTROL::FIRE) ctrl.mode = CONTROL::RESET;
		return false;
	}
	ctrl.mode = CONTROL::FIRE;
	ctrl.chassis.speedx = 0;
	ctrl.chassis.speedy = 0;
	ctrl.chassis.speedz = 0;
	// 横向通道仍保留原摩擦轮开关；它不参与供弹轮的竖直速度目标。
	ctrl.shooter.openRub = abs(rc.ch[0]) > 330;
	return true;
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
