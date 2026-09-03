#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "label.h"

PARAMETER& PARAMETER::Init()
{
	/*
	 * Pitch 使用连续位置。
	 * pitch_min/max 是相对于上电 home 的编码器偏移。
	 */
	pitch_min = -90000;
	pitch_max = 805000;

	initial_pitch = 4096;
	initial_yaw = 4900;

	imu_pitch_max = 18;
	imu_pitch_min = 16;

	ace_speed = 1000;
	max_speed = 3000;
	rota_speed = 3000;

	pitch_speed = 8;
	yaw_speed = 2;

	return *this;
}




/*
定义任务句柄
*/
TaskHandle_t StartTask_Handler;
TaskHandle_t LedTask_Handler;
TaskHandle_t DecodeTask_Handler;
TaskHandle_t ControlTask_Handler;
TaskHandle_t MotorTask_Handler;
TaskHandle_t CanTxTask_Handler;

