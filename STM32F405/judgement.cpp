#include "judgement.h"
#include "label.h"
#include <cstdarg>
#include "imu.h"
#include "control.h"
#include "xuc.h"
#include "RC.h"
#include "supercap.h"
void Judgement::BuffData()
{
	if (m_uart->updateFlag)
	{
		m_uart->updateFlag = false;
		pd_Rx = xQueueReceive(*queueHandler, m_uartrx, NULL);
		m_readnum = m_uart->dataDmaNum;
		if ((m_whand + m_readnum) < (m_FIFO + BUFSIZE))
		{
			memcpy(m_whand, m_uartrx, m_readnum);
			m_whand = m_whand + m_readnum;
		}
		else if ((m_whand + m_readnum) == (m_FIFO + BUFSIZE))
		{
			memcpy(m_whand, m_uartrx, m_readnum);
			m_whand = m_FIFO;
		}
		else
		{
			const uint8_t left_size = m_FIFO + BUFSIZE - m_whand;
			memcpy(m_whand, m_uartrx, left_size);
			m_whand = m_FIFO;
			memcpy(m_whand, m_uartrx + left_size, m_readnum - left_size);
			m_whand = m_FIFO + m_readnum - left_size;
		}
		m_leftsize = m_leftsize + m_readnum;

		/*supercap.Txsuper.limit = judgement.data.robot_status_t.chassis_power_limit;
		supercap.Txsuper.buffer = judgement.data.power_heat_data_t.chassis_power_buffer;*/
	}

}

void Judgement::Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base)
{
	huart->Init(uart_base, baud).DMARxInit();
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;

}


void Judgement::GetData(void)
{
	if (Transmit(5, m_frame) == 0)return;
	while (m_frame[0] != 0xA5)
	{
		memcpy(m_frame, m_frame + 1, 4);
		if (Transmit(1, m_frame + 4) == 0)return;
	}
	if (VerifyCRC8CheckSum(m_frame, 5))
	{
		const uint16_t datalength = static_cast<uint16_t>((m_frame[1]) | (m_frame[2] << 8));
		if (Transmit(datalength + 4, m_frame + 5) == 0)return;
		if (VerifyCRC16CheckSum(m_frame, (datalength + 9)))Decode(m_frame);
	}
}

void Judgement::DisplayStaticUI()
{
	string_data_struct_t staticStringUI;
	switch (count % 100)
	{
	case 0:
	{
		char UIRPData[5] = {};

		UIRPData[0] = 'G';
		UIRPData[1] = 'E';
		UIRPData[2] = 'A';
		UIRPData[3] = 'R';

		Char_Draw(&staticStringUI, "PI", UI_Graph_ADD, 9, UI_Color_Green, 30, 5, 3, 60, 820, UIRPData);//RCPC

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 10:
	{
		break;
	}
	case 20:
	{
		char UICapData[5] = {};
		UICapData[0] = 'C';
		UICapData[1] = 'A';
		UICapData[2] = 'P';

		Char_Draw(&staticStringUI, "CA", UI_Graph_ADD, 7, UI_Color_Green, 30, 3, 3, 60, 760, UICapData);//CAP

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 30:
	{
		char UIModeData[5] = {};
		UIModeData[0] = 'M';
		UIModeData[1] = 'O';
		UIModeData[2] = 'D';
		UIModeData[3] = 'E';

		Char_Draw(&staticStringUI, "MO", UI_Graph_ADD, 6, UI_Color_Green, 30, 5, 3, 60, 700, UIModeData);//MODE

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 40:
	{
		char UICaptureData[7] = {};

		UICaptureData[0] = 'O';
		UICaptureData[1] = 'P';
		UICaptureData[2] = 'E';
		UICaptureData[3] = 'N';
		UICaptureData[4] = 'R';
		UICaptureData[5] = 'U';
		UICaptureData[6] = 'B';

		Char_Draw(&staticStringUI, "CPT", UI_Graph_ADD, 5, UI_Color_Green, 30, 7, 3, 60, 640, UICaptureData);//CAPTURE

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 50:
	{
		//char UICapDisplayData1[3] = {};
		//UICapDisplayData1[0] = '1';
		//UICapDisplayData1[1] = '3';
		//UICapDisplayData1[2] = 'V';

		//Char_Draw(&staticStringUI, "C1", UI_Graph_ADD, 4, UI_Color_Purplish_red, 20, 3, 2, 580, 70, UICapDisplayData1);//13V

		//Char_ReFresh(&staticStringUI);
		break;
	}
	case 60:
	{
		//char UICapDisplayData2[3] = {};
		//UICapDisplayData2[0] = '1';
		//UICapDisplayData2[1] = '9';
		//UICapDisplayData2[2] = 'V';

		//Char_Draw(&staticStringUI, "C2", UI_Graph_ADD, 4, UI_Color_Yellow, 20, 3, 2, 1012, 70, UICapDisplayData2);//19V

		//Char_ReFresh(&staticStringUI);
		break;
	}
	case 70:
	{
		char UICapDisplayData3[3] = {};
		UICapDisplayData3[0] = '2';
		UICapDisplayData3[1] = '4';
		UICapDisplayData3[2] = 'V';

		Char_Draw(&staticStringUI, "C2", UI_Graph_ADD, 3, UI_Color_Green, 20, 3, 2, 1300, 70, UICapDisplayData3);//23V

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 80:
	{
		graphic_data_struct_t staticGraohUI3[5] = {};
		Rectangle_Draw(&staticGraohUI3[0], "CR", UI_Graph_ADD, 3, UI_Color_Green, 2, 600, 100, 1320, 150);
		LineDraw(&staticGraohUI3[1], "CL", UI_Graph_ADD, 3, UI_Color_Yellow, 2, 1032, 100, 1032, 150);
		LineDraw(&staticGraohUI3[2], "LL1", UI_Graph_ADD, 3, UI_Color_Main, 2, 480, 100, 700, 400);
		LineDraw(&staticGraohUI3[3], "LL2", UI_Graph_ADD, 3, UI_Color_Main, 2, 1440, 100, 1220, 400);

		UI_ReFresh(5, staticGraohUI3);
		break;
	}
	case 90:
	{
		graphic_data_struct_t staticGraohUI2[7] = {};

		LineDraw(&staticGraohUI2[0], "L1", UI_Graph_ADD, 2, UI_Color_Purplish_red, 2, 1011, 280, 1011, 600);
		//LineDraw(&staticGraohUI2[1], "L2", UI_Graph_ADD, 2, UI_Color_Purplish_red, 2, 880, 400, 1040, 400);
		//LineDraw(&staticGraohUI2[2], "L3", UI_Graph_ADD, 2, UI_Color_Purplish_red, 2, 900, 350, 1020, 350);
		Circle_Draw(&staticGraohUI2[3], "R1", UI_Graph_ADD, 2, UI_Color_Purplish_red, 3, 1011, 505, 10);
		///LineDraw(&staticGraohUI2[4], "L4", UI_Graph_ADD, 2, UI_Color_Purplish_red, 2, 850, 450, 1070, 450);
		//LineDraw(&staticGraohUI2[5], "L5", UI_Graph_ADD, 2, UI_Color_Purplish_red, 2, 880, 540, 1040, 540);

		UI_ReFresh(7, staticGraohUI2);
		break;
	}
	default:
		break;
	}
}

void Judgement::DisplayRP(int flag)
{
	char RP[2] = {};
	string_data_struct_t RPData;
	switch (flag)
	{
	case 0:
		RP[0] = 'D';
		RP[1] = '0';
		break;
	case 1:
		RP[0] = 'D';
		RP[1] = '1';
		break;
	case 2:
		RP[0] = 'D';
		RP[1] = '2';
		break;
	default:
		break;
	}
	if (!graphInit)
	{
		Char_Draw(&RPData, "aa", UI_Graph_ADD, 5, UI_Color_Green, 30, 2, 3, 280, 820, RP);
	}
	else
	{
		Char_Draw(&RPData, "aa", UI_Graph_Change, 5, UI_Color_Green, 30, 2, 3, 280, 820, RP);
	}

	Char_ReFresh(&RPData);
}

void Judgement::DisplayCapState(uint8_t capState)
{
	char capStateChar[5] = {};
	string_data_struct_t capStateData;
	if (capState == WORKING)
	{
		capStateChar[0] = 'W';
		capStateChar[1] = 'O';
		capStateChar[2] = 'R';
		capStateChar[3] = 'K';
	}
	else if (capState == DISCHARGE)
	{
		capStateChar[0] = 'D';
		capStateChar[1] = 'I';
		capStateChar[2] = 'S';
		capStateChar[3] = 'C';
		capStateChar[4] = 'H';
	}
	else if (capState == SHUT)
	{
		capStateChar[0] = 'S';
		capStateChar[1] = 'H';
		capStateChar[2] = 'U';
		capStateChar[3] = 'T';
	}

	if (!graphInit)
	{
		Char_Draw(&capStateData, "CO1", UI_Graph_ADD, 7, UI_Color_Green, 30, 5, 3, 280, 760, capStateChar);
	}
	else
	{
		Char_Draw(&capStateData, "CO1", UI_Graph_Change, 7, UI_Color_Green, 30, 5, 3, 280, 760, capStateChar);
	}

	Char_ReFresh(&capStateData);
}

void Judgement::DisplpayMode(uint8_t mode)
{
	CONTROL::MODE _mode;
	_mode = (CONTROL::MODE)mode;

	char modeChar[7] = {};
	string_data_struct_t modeData;

	switch (_mode)
	{
	case CONTROL::MODE::LOCK:
	{
		modeChar[0] = 'L';
		modeChar[1] = 'O';
		modeChar[2] = 'C';
		modeChar[3] = 'K';
	}
	break;
	case CONTROL::MODE::FOLLOW:
	{
		modeChar[0] = 'F';
		modeChar[1] = 'O';
		modeChar[2] = 'L';
		modeChar[3] = 'L';
		modeChar[4] = 'O';
		modeChar[5] = 'W';
	}
	break;
	case CONTROL::MODE::ROTATION:
	{
		modeChar[0] = 'R';
		modeChar[1] = 'O';
		modeChar[2] = 'T';
		modeChar[3] = 'A';
	}
	break;
	case CONTROL::MODE::MANUAL_YAW:
	{
		modeChar[0] = 'S';
		modeChar[1] = 'E';
		modeChar[2] = 'P';
	}
	break;
	default:
		break;
	};
	if (!graphInit)
	{
		Char_Draw(&modeData, "MD", UI_Graph_ADD, 6, UI_Color_Green, 30, 7, 3, 280, 700, modeChar);
	}
	else
	{
		Char_Draw(&modeData, "MD", UI_Graph_Change, 6, UI_Color_Green, 30, 7, 3, 280, 700, modeChar);
	}

	Char_ReFresh(&modeData);
}

void Judgement::DisplayCapture(bool isCapture)
{
	char captureChar[5] = {};
	string_data_struct_t captureData;
	if (isCapture)
	{
		captureChar[0] = 'T';
		captureChar[1] = 'R';
		captureChar[2] = 'U';
		captureChar[3] = 'E';
	}
	else
	{
		captureChar[0] = 'F';
		captureChar[1] = 'A';
		captureChar[2] = 'L';
		captureChar[3] = 'S';
		captureChar[4] = 'E';
	}
	if (!graphInit)
	{
		Char_Draw(&captureData, "CO", UI_Graph_ADD, 5, UI_Color_Green, 30, 5, 3, 280, 640, captureChar);
	}
	else
	{
		Char_Draw(&captureData, "CO", UI_Graph_Change, 5, UI_Color_Green, 30, 5, 3, 280, 640, captureChar);
	}

	Char_ReFresh(&captureData);
}

void Judgement::DisplayCapVoltage(float capVoltage)
{
	uint32_t voltagePos{};
	graphic_data_struct_t voltageData;

	voltagePos = (capVoltage) * 720 / (2000.f) + 600;

	if (voltagePos < 600)
	{
		voltagePos = 600;
	}
	else if (voltagePos > 1320)
	{
		voltagePos = 1320;
	}

	if (!graphInit)
	{
		LineDraw(&voltageData, "VD", UI_Graph_ADD, 3, UI_Color_Yellow, 50, 600, 125, voltagePos, 125);
	}
	else
	{
		LineDraw(&voltageData, "VD", UI_Graph_Change, 3, UI_Color_Yellow, 50, 600, 125, voltagePos, 125);
	}

	UI_ReFresh(1, &voltageData);
}

void Judgement::SendData(void)
{

	robotId = data.robot_status_t.robot_id;
	clientId = robotId | 0x100;

	if (count < 200)
	{
		DisplayStaticUI();
	}
	else
	{
		switch (count % 20)
		{
		case 0:
		{
			DisplayRP(rc.gear);
			break;
		}
		case 4:
		{
			DisplayCapState(supercap.Txsuper.state);
			break;
		}
		case 8:
		{
			DisplayCapVoltage(supercap.Rxsuper.cap_energy);
			break;
		}
		case 12:
		{
			DisplayCapture(ctrl.shooter.openRub);
			break;
		}
		case 16:
		{
			//DisplpayMode(ctrl.mode[now]);
			break;
		}

		default:
			break;
		}
	}
	if (count > 2000)
	{
		graphInit = true;
	}
	count++;

}

void Judgement::Decode(uint8_t* m_frame)//未完全
{
	const uint16_t cmdID = static_cast<uint16_t>(m_frame[5] | m_frame[6] << 8);
	uint8_t* rawdata = &m_frame[7];
	switch (cmdID)
	{
	case 0x0001:
		data.game_status_t.game_type = static_cast<uint8_t>(rawdata[0] & 0x0F);
		data.game_status_t.game_progress = static_cast<uint8_t>(rawdata[0] >> 4);
		data.game_status_t.stage_remain_time = static_cast<uint16_t>(rawdata[1] | rawdata[2] << 8);
		break;
		//data.ext_game_status_t.SyncTimeStamp

	case 0x0002:
		data.game_result_t.winner = rawdata[0];
		break;

	case 0x0003:
		data.game_robot_HP_t.red_1_robot_HP = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.game_robot_HP_t.red_2_robot_HP = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.game_robot_HP_t.red_3_robot_HP = static_cast<uint16_t>(rawdata[4] | rawdata[5] << 8);
		data.game_robot_HP_t.red_4_robot_HP = static_cast<uint16_t>(rawdata[6] | rawdata[7] << 8);
		data.game_robot_HP_t.red_5_robot_HP = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.game_robot_HP_t.red_7_robot_HP = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.game_robot_HP_t.red_outpost_HP = static_cast<uint16_t>(rawdata[12] | rawdata[13] << 8);
		data.game_robot_HP_t.red_base_HP = static_cast<uint16_t>(rawdata[14] | rawdata[15] << 8);
		data.game_robot_HP_t.blue_1_robot_HP = static_cast<uint16_t>(rawdata[16] | rawdata[17] << 8);
		data.game_robot_HP_t.blue_2_robot_HP = static_cast<uint16_t>(rawdata[18] | rawdata[19] << 8);
		data.game_robot_HP_t.blue_3_robot_HP = static_cast<uint16_t>(rawdata[20] | rawdata[21] << 8);
		data.game_robot_HP_t.blue_4_robot_HP = static_cast<uint16_t>(rawdata[22] | rawdata[23] << 8);
		data.game_robot_HP_t.blue_5_robot_HP = static_cast<uint16_t>(rawdata[24] | rawdata[25] << 8);
		data.game_robot_HP_t.blue_7_robot_HP = static_cast<uint16_t>(rawdata[26] | rawdata[27] << 8);
		data.game_robot_HP_t.blue_outpost_HP = static_cast<uint16_t>(rawdata[28] | rawdata[29] << 8);
		data.game_robot_HP_t.blue_base_HP = static_cast<uint16_t>(rawdata[30] | rawdata[31] << 8);
		break;
	case 0x0101:
		data.event_data_t.event_data = u32_to_float(&rawdata[0]);
		break;
	case 0x0102:
		data.ext_supply_projectile_action_t.reserved = rawdata[0];
		data.ext_supply_projectile_action_t.supply_robot_id = rawdata[1];
		data.ext_supply_projectile_action_t.supply_projectile_step = rawdata[2];
		data.ext_supply_projectile_action_t.supply_projectile_num = rawdata[3];
		break;
	case 0x0104:
		data.referee_warning_t.level = rawdata[0];
		data.referee_warning_t.foul_robot_id = rawdata[1];
		data.referee_warning_t.count = rawdata[2];
		break;
	case 0x0105:
		data.dart_dart_info_t.dart_info = rawdata[0];
		data.dart_dart_info_t.dart_remaining_time = rawdata[1];
		break;
	case 0x0201:
		data.robot_status_t.robot_id = rawdata[0];
		judgementready = true;
		data.robot_status_t.robot_level = rawdata[1];
		data.robot_status_t.current_HP = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.robot_status_t.maximum_HP = static_cast<uint16_t>(rawdata[4] | rawdata[5] << 8);
		data.robot_status_t.shooter_barrel_cooling_value = static_cast<uint16_t>(rawdata[6] | rawdata[7] << 8);
		data.robot_status_t.shooter_barrel_heat_limit = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.robot_status_t.chassis_power_limit = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.robot_status_t.power_management_gimbal_output = static_cast<uint16_t>(rawdata[12] & 0x01);
		data.robot_status_t.power_management_chassis_output = static_cast<uint16_t>(rawdata[12] & 0x02) >> 1;
		data.robot_status_t.power_management_shooter_output = static_cast<uint16_t>(rawdata[12] & 0x04) >> 2;


		break;
	case 0x0202:
		powerheatready = true;
		data.power_heat_data_t.chassis_voltage = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.power_heat_data_t.chassis_current = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.power_heat_data_t.chassis_power = u32_to_float(&rawdata[4]);
		data.power_heat_data_t.chassis_power_buffer = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.power_heat_data_t.shooter_17mm_1_barrel_heat = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.power_heat_data_t.shooter_17mm_2_barrel_heat = static_cast<uint16_t>(rawdata[12] | rawdata[13] << 8);
		data.power_heat_data_t.shooter_42mm_barrel_heat = static_cast<uint16_t>(rawdata[14] | rawdata[15] << 8);

		supercap.Txsuper.limit = judgement.data.robot_status_t.chassis_power_limit;
		supercap.Txsuper.buffer = judgement.data.power_heat_data_t.chassis_power_buffer;
		//ssupercap.update_cnt++;

		break;
	case 0x0203:
		data.robot_pos_t.x = u32_to_float(&rawdata[0]);
		data.robot_pos_t.y = u32_to_float(&rawdata[4]);
		data.robot_pos_t.angle = u32_to_float(&rawdata[12]);
		break;
	case 0x0204:
		data.buff_t.recovery_buff = rawdata[0];
		data.buff_t.cooling_buff = rawdata[1];
		data.buff_t.defence_buff = rawdata[2];
		data.buff_t.vulnerability_buff = rawdata[3];
		data.buff_t.attack_buff = rawdata[4];
		break;
	case 0x0205:
		data.air_support_data_t.airforce_status = rawdata[0];
		data.air_support_data_t.time_remain = rawdata[1];
		break;
	case 0x0206:
		data.hurt_data_t.armor_id = static_cast<uint8_t>(rawdata[0] & 0x0F);
		data.hurt_data_t.HP_deduction_reason = static_cast<uint8_t>(rawdata[0] >> 4);
		break;
	case 0x0207:
		data.shoot_data_t.bullet_type = rawdata[0];
		data.shoot_data_t.shooter_number = rawdata[1];
		data.shoot_data_t.bullet_freq = rawdata[2];
		data.shoot_data_t.bullet_speed = u32_to_float(&rawdata[3]);
		if (prebulletspd != data.shoot_data_t.bullet_speed)
		{
			nBullet++;
			prebulletspd = data.shoot_data_t.bullet_speed;
		}
		break;
	case 0x0208:
		data.projectile_allowance_t.projectile_allowance_17mm = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.projectile_allowance_t.projectile_allowance_42mm = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.projectile_allowance_t.remaining_gold_coin = static_cast <uint16_t>(rawdata[4] | rawdata[5] << 8);
		break;

	case 0x0209:
		data.rfid_status_t.rfid_status = u32_to_float(&rawdata[0]);
		baseRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & 0x01);
		highlandRFID = static_cast<uint8_t>((data.rfid_status_t.rfid_status & 0x02) | data.rfid_status_t.rfid_status & 0x04);
		energyRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 7));
		feipoRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 8));
		outpostRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 12));
		resourseRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 16));
		break;
	case 0x020A:
		data.dart_client_cmd_t.dart_launch_opening_status = rawdata[0];
		data.dart_client_cmd_t.reserved = rawdata[1];
		data.dart_client_cmd_t.target_change_time = rawdata[2];
		data.dart_client_cmd_t.latest_launch_cmd_time = rawdata[3];
		break;
	case 0x020B:
		data.ground_robot_position_t.hero_x = u32_to_float(&rawdata[0]);
		data.ground_robot_position_t.hero_y = u32_to_float(&rawdata[4]);
		data.ground_robot_position_t.engineer_x = u32_to_float(&rawdata[8]);
		data.ground_robot_position_t.engineer_y = u32_to_float(&rawdata[12]);
		data.ground_robot_position_t.standard_3_x = u32_to_float(&rawdata[16]);
		data.ground_robot_position_t.standard_3_y = u32_to_float(&rawdata[20]);
		data.ground_robot_position_t.standard_4_x = u32_to_float(&rawdata[24]);
		data.ground_robot_position_t.standard_4_y = u32_to_float(&rawdata[28]);
		data.ground_robot_position_t.standard_5_x = u32_to_float(&rawdata[32]);
		data.ground_robot_position_t.standard_5_y = u32_to_float(&rawdata[36]);
		break;

	default:
		break;
	}
}

bool Judgement::Transmit(uint32_t read_size, uint8_t* plate)
{
	if (m_leftsize < read_size)return false;

	if ((m_rhand + read_size) < (m_FIFO + BUFSIZE))
	{
		memcpy(plate, m_rhand, read_size);
		m_rhand = m_rhand + read_size;
	}
	else if ((m_rhand + read_size) == (m_FIFO + BUFSIZE))
	{
		memcpy(plate, m_rhand, read_size);
		m_rhand = m_FIFO;
	}
	else
	{
		const uint8_t left_size = m_FIFO + BUFSIZE - m_rhand;
		memcpy(plate, m_rhand, left_size);
		memcpy(plate + left_size, m_rhand = m_FIFO, read_size - left_size);
		m_rhand = m_FIFO + read_size - left_size;
	}
	m_leftsize = m_leftsize - read_size;
	return true;
}


/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    开始坐标
		End_x、End_y   结束坐标
**********************************************************************************************************/
void Judgement::LineDraw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
	uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Line;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->end_x = End_x;
	image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    开始坐标
		End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
void Judgement::Rectangle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
	uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Rectangle;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->end_x = End_x;
	image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    圆心坐标
		Graph_Radius  图形半径
**********************************************************************************************************/
void Judgement::Circle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
	uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Circle;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->radius = Graph_Radius;
}


/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_StartAngle,Graph_EndAngle    开始，终止角度
		Start_y,Start_y    圆心坐标
		x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
void Judgement::Arc_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
	uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t x_Length, uint32_t y_Length)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Arc;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->start_angle = Graph_StartAngle;
	image->end_angle = Graph_EndAngle;
	image->end_x = x_Length;
	image->end_y = y_Length;
}


/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_Size     字号
		Graph_Digit    小数位数
		Start_x、Start_x    开始坐标
		Graph_Float   要显示的变量
**********************************************************************************************************/
void Judgement::Float_Draw(float_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, \
	uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float)
{
	int i;
	for (i = 0; i < 2 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Float;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->start_angle = Graph_Size;
	image->end_angle = Graph_Digit;

	int32_t temp1 = Graph_Float * 1000;
	int32_t temp2 = temp1 / 1024;
	image->end_x = temp2;

	image->radius = temp1 - temp2 * 1024;//1->1.024*e-3
}


/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_Size     字号
		Graph_Digit    字符个数
		Start_x、Start_x    开始坐标
		*Char_Data          待发送字符串开始地址
**********************************************************************************************************/
void Judgement::Char_Draw(string_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, \
	uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char* Char_Data)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->Graph_Control.figure_name[2 - i] = imagename[i];
	image->Graph_Control.figure_tpye = UI_Graph_Char;
	image->Graph_Control.operate_tpye = Graph_Operate;
	image->Graph_Control.layer = Graph_Layer;
	image->Graph_Control.color = Graph_Color;
	image->Graph_Control.width = Graph_Width;
	image->Graph_Control.start_x = Start_x;
	image->Graph_Control.start_y = Start_y;
	image->Graph_Control.start_angle = Graph_Size;
	image->Graph_Control.end_angle = Graph_Digit;

	for (i = 0; i < Graph_Digit; i++)
	{
		image->show_Data[i] = *Char_Data;
		Char_Data++;
	}
}

/************************************************UI删除函数（使更改生效）*********************************
**参数： cnt   图形个数
		 ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
void Judgement::UIDelete(uint8_t deleteOperator, uint8_t deleteLayer)
{
	uint16_t dataLength;
	CommunatianData_graphic_t UIDeleteData;

	UIDeleteData.txFrameHeader.sof = 0xA5;
	UIDeleteData.txFrameHeader.data_length = 8;
	UIDeleteData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &UIDeleteData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	UIDeleteData.CMD = UI_CMD_Robo_Exchange;
	UIDeleteData.txID.data_cmd_id = UI_Data_ID_Del;
	UIDeleteData.txID.receiver_ID = clientId;
	UIDeleteData.txID.sender_ID = robotId;

	memcpy(m_uarttx + 5, (uint8_t*)&UIDeleteData.CMD, 8);

	m_uarttx[13] = deleteOperator;
	m_uarttx[14] = deleteLayer;

	dataLength = sizeof(CommunatianData_graphic_t) + 2;

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;
}


/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
		 ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
void Judgement::UI_ReFresh(int cnt, graphic_data_struct_t* imageData)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 6 + cnt * 15;
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;
	switch (cnt)
	{
	case 1:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
		break;
	case 2:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
		break;
	case 5:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
		break;
	case 7:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
		break;
	default:
		break;
	}
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, imageData, cnt * sizeof(graphic_data_struct_t));
	dataLength = sizeof(CommunatianData_graphic_t) + cnt * sizeof(graphic_data_struct_t);

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}

void Judgement::UI_ReFresh(int cnt, float_data_struct_t* floatdata)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 6 + cnt * sizeof(graphic_data_struct_t);
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;
	switch (cnt)
	{
	case 1:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
		break;
	case 2:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
		break;
	case 5:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
		break;
	case 7:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
		break;
	default:
		break;
	}
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, floatdata, cnt * sizeof(graphic_data_struct_t));
	dataLength = sizeof(CommunatianData_graphic_t) + cnt * sizeof(graphic_data_struct_t);

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}

/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
		 ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
void Judgement::Char_ReFresh(string_data_struct_t* string_Data)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 51;
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;

	graphicData.txID.data_cmd_id = UI_Data_ID_DrawChar;
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, string_Data, sizeof(string_data_struct_t));
	dataLength = sizeof(CommunatianData_graphic_t) + sizeof(string_data_struct_t);

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}



