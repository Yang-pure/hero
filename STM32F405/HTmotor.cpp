#include "HTmotor.h"
#include "delay.h"
#include "can.h"
#include "stdio.h"

#define now 0
#define last 1
void buffer_append_int32(uint8_t* buffer, int32_t number, int16_t* index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}
void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t* index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

DMMOTOR& DMMOTOR::State_Decode(CAN hcan, uint8_t idata[][8])//接收反馈数据
{
	//浮点型数据
	//receive_data[0]=电机id
	uint8_t id = ID - 0x01;
	int direct = 0;
	int tmp_value = 0;
	error = (idata[id][0] >> 4) & 0x0F;
	tmp_value = (idata[id][1] << 8) | (idata[id][2]);//电机位置
	pos = uint_to_float(tmp_value, P_MIN, P_MAX, 16);//浮点型
	tmp_value = (idata[id][3] << 4) | (idata[id][4] >> 4);//转速
	curSpeed = uint_to_float(tmp_value, V_MIN, V_MAX, 12);//转浮点型
	tmp_value = (idata[id][5]) | ((idata[id][4] & 0x0f) << 8);
	current = uint_to_float(tmp_value, C_MIN, C_MAX, 12);
	torque = current * KT;//（力矩=电流*转矩常数，本产品转矩常数为 1.4Nm/A）
	return *this;
}


void DMMOTOR::DMmotor_transmit(uint32_t id)
{
	//CanComm_ControlCmd(can1, CMD_RESET_MODE, id + MOTOR_MODE);//电机失力
	can1.Transmit(id + MOTOR_MODE, can1.jointpdata[id - 1], 8);
}

void DMMOTOR::DMmotorinit()
{
	CanComm_ControlCmd(can2, CMD_MOTOR_MODE, MOTOR_MODE + 1);
	delay.delay_ms(1);
}

void DMMOTOR::SetTorque(float settorque)
{
	setTorque = settorque;
}

float DMMOTOR::GetPosition()
{
	return angle[now];
}

float DMMOTOR::GetSpeed()
{
	return curSpeed;
}

float DMMOTOR::GetTorque()
{
	return torque;
}

void  DMMOTOR::CanComm_ControlCmd(CAN& hcan, uint8_t cmd, uint32_t id)//使能帧
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00 };
	switch (cmd)
	{
	case CMD_MOTOR_MODE:
		buf[7] = 0xFC;//进入电机
		break;

	case CMD_RESET_MODE:
		buf[7] = 0xFD;//退出电机
		break;

	case CMD_ZERO_POSITION:
		buf[7] = 0xFE;//保存位置零点
		break;

	case CMD_CLEAR_MODE:
		buf[7] = 0xFB;//清除错误
		break;

	default:
		return; /* 直接退出函数 */
	}
	hcan.Transmit(id, buf, 8);
}

float DMMOTOR::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int DMMOTOR::float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DMMOTOR::DMmotor_Ontimer(CAN hcan, float f_kp, float f_kd, uint8_t* odata)
{
	unsigned char* P = (unsigned char*)&setPos; // 定义一个无符号字符型指针p并指向f的地址
	unsigned char* V = (unsigned char*)&setSpeed; // 定义一个无符号字符型指针v并指向f的地址
	uint8_t id = ID - 0x01;
	uint32_t p = 0, v = 0, kp = 0, kd = 0, t = 0;//位置给定，速度给定，位置比例系数，位置微分系数，转矩给定值
	/* 限制输入的参数在定义的范围内 */
	LIMIT_MIN_MAX(setPos, P_MIN, P_MAX);
	LIMIT_MIN_MAX(setSpeed, V_MIN, V_MAX);
	LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
	LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
	LIMIT_MIN_MAX(setTorque, T_MIN, T_MAX);
	switch (MOTOR_MODE)
	{
	case 0x00:
		/* 根据协议，对float参数进行转换 */
		p = float_to_uint(setPos, P_MIN, P_MAX, 16);//位置两个字节
		v = float_to_uint(setSpeed, V_MIN, V_MAX, 12);//速度12位
		kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);//比例系数12位
		kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);//速度系数12位
		t = float_to_uint(setTorque, T_MIN, T_MAX, 12);//前馈力矩（电流）
		/* 根据传输协议，把数据转换为CAN命令数据字段并存入输出缓冲区*/

		odata[0] = p >> 8;
		odata[1] = p & 0xFF;
		odata[2] = v >> 4;
		odata[3] = ((v & 0xF) << 4) | (kp >> 8);
		odata[4] = kp & 0xFF;
		odata[5] = kd >> 4;
		odata[6] = ((kd & 0xF) << 4) | (t >> 8);
		odata[7] = t & 0xff;
		break;

	case 0x100:
		odata[0] = P[0];
		odata[1] = P[1];
		odata[2] = P[2];
		odata[3] = P[3];
		odata[4] = V[0];
		odata[5] = V[1];
		odata[6] = V[2];
		odata[7] = V[3];
		///* 根据协议，对float参数进行转换 */
		//下面这种转换方式存在问题，留待以后解决，上面方法可用
		//p = float_to_uint(setPos, P_MIN, P_MAX, 32);
		//v = float_to_uint(setSpeed, V_MIN, V_MAX, 32);
		///* 根据传输协议，把数据转换为CAN命令数据字段并存入输出缓冲区*/
		//odata[id][0] = p & 0xff;
		//odata[id][1] = (p >> 8) & 0xff;
		//odata[id][2] = (p >> 16) & 0xff;
		//odata[id][3] = (p >> 24) & 0xff;
		//odata[id][4] = v & 0xff;
		//odata[id][5] = (v >>8) & 0xff;
		//odata[id][6] = (v >>16) & 0xff;
		//odata[id][7] = (v >>24) & 0xff;
		break;

	case 0x200:
		break;

	default:
		return; /* 直接退出函数 */
	}
}
