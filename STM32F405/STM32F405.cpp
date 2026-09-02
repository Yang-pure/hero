  /*
   *__/\\\_______/\\\__/\\\\____________/\\\\__/\\\________/\\\______________/\\\\\\\\\____________/\\\\\\\\\_____/\\\\\\\\\\\___
   * _\///\\\___/\\\/__\/\\\\\\________/\\\\\\_\/\\\_______\/\\\____________/\\\///////\\\_______/\\\////////____/\\\/////////\\\_
   *  ___\///\\\\\\/____\/\\\//\\\____/\\\//\\\_\/\\\_______\/\\\___________\/\\\_____\/\\\_____/\\\/____________\//\\\______\///__
   *   _____\//\\\\______\/\\\\///\\\/\\\/_\/\\\_\/\\\_______\/\\\___________\/\\\\\\\\\\\/_____/\\\_______________\////\\\_________
   *    ______\/\\\\______\/\\\__\///\\\/___\/\\\_\/\\\_______\/\\\___________\/\\\//////\\\____\/\\\__________________\////\\\______
   *     ______/\\\\\\_____\/\\\____\///_____\/\\\_\/\\\_______\/\\\___________\/\\\____\//\\\___\//\\\____________________\////\\\___
   *      ____/\\\////\\\___\/\\\_____________\/\\\_\//\\\______/\\\____________\/\\\_____\//\\\___\///\\\___________/\\\______\//\\\__
   *       __/\\\/___\///\\\_\/\\\_____________\/\\\__\///\\\\\\\\\/_____________\/\\\______\//\\\____\////\\\\\\\\\_\///\\\\\\\\\\\/___
   *        _\///_______\///__\///_____________\///_____\/////////_______________\///________\///________\/////////____\///////////_____
  */

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "HTmotor.h"
#include "Power_read.h"

Motor can1_motor[CAN1_MOTOR_NUM] = {
	Motor(M3508,SPD,chassis, ID5, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M3508,SPD,chassis, ID6, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M3508,SPD,chassis, ID7, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M3508,SPD,chassis, ID8, PID(10.f, 0.0f, 1.5f,0.f))
};
Motor can2_motor[CAN2_MOTOR_NUM] = {
	Motor(M3508,SPD,shooter, ID1, PID(0.f, 0.0f, 1.5f,0.f)),
	Motor(M3508,SPD,shooter, ID2, PID(0.f, 0.0f, 1.5f,0.f)),
	Motor(M3508,SPD,shooter, ID3, PID(0.f, 0.0f, 1.5f,0.f)),
	Motor(M6020,POS,pantile, ID6, PID(4.3f, 0.1f, 1.f,0.f),PID(0.8f, 0.05f, 5.0f,0.f)),
	Motor(M3508,POS,supply, ID5, PID(0.f, 0.0f, 1.5f,0.f)),
	Motor(M3508,POS,pantile, ID4, PID(0.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.f, 0.f,0.f))
};
DMMOTOR DMmotor[4] = {
	DMMOTOR(0x01, P_S, L_F),
	DMMOTOR(0x02, P_S, L_F),
	DMMOTOR(0x03, P_S, L_F),
	DMMOTOR(0x04, P_S, L_F),
};


CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile;
DELAY delay;
RC rc;
POWER power;
LED led1, led2, led3, led4;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;


int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init();
	can1.Init(CAN1);
	//HAL_CAN_Start(&hcan1);
	//HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can2.Init(CAN2);
	timer.Init(BASE, TIM3, 1000).BaseInit();
	imu_pantile.Init(&uart2, USART2, 115200, CH010);
	rc.Init(&uart1, USART1, 100000);
	power.Init(&uart5,UART5,9600);

	para.Init();
	ctrl.Init(std::vector<Motor*>{
		&can1_motor[0], // 底盘 ID5
			& can1_motor[1], // 底盘 ID6
			& can1_motor[2], // 底盘 ID7
			& can1_motor[3], // 底盘 ID8

			& can2_motor[0], // 摩擦轮 ID1
			& can2_motor[1], // 摩擦轮 ID2
			& can2_motor[2], // 摩擦轮 ID3

			& can2_motor[3], // ID4 M6020，云台 YAW
			& can2_motor[4], // ID5 M3508，拨弹 supply
			& can2_motor[5], // ID6 M3508，云台 PITCH
	});
	//});
	//ctrl.Init(std::vector<Motor*>{
	//		& can1_motor[0],
	//		& can1_motor[1],	//底盘四个id5~id8
	//		& can1_motor[2],
	//		& can1_motor[3], 
	//});
	//ctrl.Init(std::vector<Motor*>{
	//		& can2_motor[0],
	//		& can2_motor[1],	//1~3摩擦轮3508
	//		& can2_motor[2],	
	//		& can2_motor[3],
	//		& can2_motor[4],	// PITCH，目前不用,3508
	//		& can2_motor[5],	//云台6020
	//});

	task.Init();
	for (;;)
		;
}





