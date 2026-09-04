#include "xuc.h"
#include "label.h"
#include "imu.h"
#include "CRC.h"
#include "math.h"
#include "RC.h"
#include "task.h"

void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	frame = m_uart->m_uartrx;
	queue_handler = &huart->UartQueueHandler;

	autoaim_controller[0].m_Kp = 0.005f;
	autoaim_controller[0].m_Td = 0.004f;

	autoaim_controller[1].m_Kp = 0.0014f;
	autoaim_controller[1].m_Td = 0.001f;
}

void XUC::Decode()
{
	pd_Rx = xQueueReceive((m_uart->UartQueueHandler), m_frame, 0);

	// 只处理这一次真正收到的新帧，避免旧m_frame被重复计成“连续50帧”。
	if (pd_Rx != pdTRUE)
	{
		// 通信中断超过100ms时撤销锁定，避免继续使用陈旧目标。
		if (last_valid_rx_tick != 0 &&
			(xTaskGetTickCount() - last_valid_rx_tick) > pdMS_TO_TICKS(100))
		{
			test_cont = 0;
			track_flag = false;
		}
		return;
	}

	if (m_frame[0] != 0xA5)
	{
		if (last_valid_rx_tick != 0 &&
			(xTaskGetTickCount() - last_valid_rx_tick) > pdMS_TO_TICKS(100))
		{
			test_cont = 0;
			track_flag = false;
		}
		return;
	}

	/*
	 * Protocol v2 is 31 bytes:
	 *   bytes 25..28 = complete v_y float
	 *   bytes 29..30 = CRC16 over bytes 0..28
	 *
	 * The historical 29-byte frame put CRC at 27..28, overlapping v_y.
	 * Accept it temporarily so an older NUC can still drive yaw/pitch, but do
	 * not use its corrupted v_y.  New NUC software must send the 31-byte frame.
	 */
	const bool is_v2_frame =
		verifyCRC16CheckSum(m_frame, CONTROL_FRAME_LEN);
	const bool is_legacy_frame =
		!is_v2_frame &&
		verifyCRC16CheckSum(m_frame, LEGACY_CONTROL_FRAME_LEN);

	if (!is_v2_frame && !is_legacy_frame)
	{
		if (last_valid_rx_tick != 0 &&
			(xTaskGetTickCount() - last_valid_rx_tick) > pdMS_TO_TICKS(100))
		{
			test_cont = 0;
			track_flag = false;
		}
		return;
	}

	last_valid_rx_tick = xTaskGetTickCount();

	// 保存上一帧yaw后再解析本帧，保证yaw_spd使用相邻两帧计算。
	yaw_pre = yaw;

	xuc.pitch = u8_to_float(m_frame + 1) * PI / 180.f;
	xuc.yaw = u8_to_float(m_frame + 5);
	xuc.yaw_diff = u8_to_float(m_frame + 9);
	xuc.pitch_diff = u8_to_float(m_frame + 13) * PI / 180.f;
	xuc.distance = u8_to_float(m_frame + 17);
	xuc.fireadvice = m_frame[21] & 0x01;
	// v_y is only trustworthy in protocol v2.  It is not required by the
	// current yaw/pitch auto-aim loop, so legacy frames remain safe to use.
	xuc.v_y = is_v2_frame ? u8_to_float(m_frame + 25) : 0.0f;

	yaw_spd = ((yaw - yaw_pre) / 0.004f) * 2 * PI / 60.f;

	if (distance > 0.f)
	{
		if (test_cont < 50)
			test_cont++;
	}
	else
	{
		test_cont = 0;
	}

	track_flag = (test_cont >= 50);
}

void XUC::Encode()
{
	own_color = judgement.data.robot_status_t.robot_id <= 7 ? RED : BLUE;
	//TxPacket TxNuc;  // 创建一个数据包实例

	TxNuc.header = 0x5A;
	TxNuc.detect_color = !own_color;
	TxNuc.reset_tracker = 0;
	TxNuc.reserved = 15;
	TxNuc.roll = imu_pantile.GetAngleRoll();
	TxNuc.pitch = imu_pantile.GetAnglePitch();
	TxNuc.yaw = imu_pantile.GetAngleYaw();
	TxNuc.aim_x = aim_x;
	TxNuc.aim_y = aim_y;
	TxNuc.aim_z = aim_z;
	TxNuc.checksum = 0;  // 初始化校验和为0

	// 计算数据包的总大小
	int packet_size = sizeof(TxNuc);

	// 将数据包复制到发送缓冲区
	memcpy(tx_data, &TxNuc, packet_size);

	// 计算并附加 CRC16 校验和
	appendCRC16CheckSum(tx_data, packet_size);

	// 发送数据
	m_uart->UARTTransmit(tx_data, packet_size);
}

uint16_t XUC::getCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t ch_data;

	if (pchMessage == nullptr) return 0xFFFF;
	while (dwLength--) {
		ch_data = *pchMessage++;
		(wCRC) =
			((uint16_t)(wCRC) >> 8) ^ CRC_TAB[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
	}

	return wCRC;
}

uint32_t XUC::verifyCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_expected = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return false;

	w_expected = getCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);
	return (
		(w_expected & 0xff) == pchMessage[dwLength - 2] &&
		((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void XUC::appendCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_crc = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return;

	w_crc = getCRC16CheckSum(reinterpret_cast<uint8_t*>(pchMessage), dwLength - 2, CRC16_INIT);

	pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}
