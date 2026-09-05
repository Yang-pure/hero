#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"
extern float Kp = 10;
extern float Kd = 0.6;
extern int start_flag;
// Live Watch观测变量
volatile float yaw_speed_plot = 0.0f;
volatile float yaw_angle_plot = 0.0f;

enum DM_INIT_STATE
{
    DM_DISABLE_ALL,
    DM_QUERY_POSITION,
    DM_SEND_HOLD_TARGET,
    DM_ENABLE_ALL,
    DM_WAIT_ENABLE,
    DM_READY,
    DM_FAULT
};

DM_INIT_STATE dm_init_state = DM_DISABLE_ALL;
uint8_t dm_init_index = 0;
uint32_t dm_disable_rx_snapshot[4]{};
uint32_t dm_enable_rx_snapshot[4]{};

void TASK::Init()
{
	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建任务

	xTaskCreate((TaskFunction_t)ArmTask,
		(const char*)"ArmTask",
		(uint16_t)LED_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)LED_TASK_PRIO,
		(TaskHandle_t*)&LedTask_Handler);

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&DecodeTask_Handler);

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&MotorTask_Handler);

	xTaskCreate((TaskFunction_t)CanTransimtTask,
		(const char*)"CanTransimtTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&CanTxTask_Handler);

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&ControlTask_Handler);

	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}
int CNT = 0;
void MotorUpdateTask(void* pvParameters)
{
	
	while (1)
	{
	TickType_t xlastWakeTime = xTaskGetTickCount();
	
		for (auto& motor : can1_motor)motor.Ontimer(can1.data, can1.temp_data);

        for (auto& motor : can2_motor)
        {
            motor.Ontimer(can2.data, can2.temp_data);
        }

        // CAN2数组下标3是Yaw电机
        // 两个变量在同一个电机控制周期内更新
        yaw_speed_plot =
            static_cast<float>(can2_motor[3].curspeed);

        yaw_angle_plot =
            static_cast<float>(can2_motor[3].angle[now]) / 400.0f;

        for (auto& dm : DMmotor)
        {
            uint8_t index = dm.ID - 1;

            if (can1.dm_rx_count[index] > 0)
            {
                dm.State_Decode(can1, can1.jointidata);
                
                if (!dm.hold_position_captured)
                {
                    dm.setPos = dm.pos;
                    dm.setSpeed = 0.5f;
                    dm.hold_position_captured = true;
                }
                dm.DMmotor_Ontimer(can1, dm.Kp, dm.Kd, can1.jointpdata[dm.ID - 1]);
            }
           
        }


	vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));//开始执行该任务之后1ms再执行该任务
}
}

void CanTransimtTask(void* pvParameters)
{
	while (true)
	{

		TickType_t xlastWakeTime1 = xTaskGetTickCount();

		switch ((timer.counter++) % 3)
		{
		case 0:
            if (!ctrl.init_DM)
            {
                switch (dm_init_state)
                {
                case DM_DISABLE_ALL:
                    DMmotor[dm_init_index].CanComm_ControlCmd(can1, CMD_RESET_MODE, MOTOR_MODE + DMmotor[dm_init_index].ID);

                    dm_init_index++;

                    if (dm_init_index >= 4)
                    {
                        dm_init_index = 0;
                        dm_init_state = DM_QUERY_POSITION;
                    }
                    break;

                case DM_QUERY_POSITION:
                {
                    bool all_ready = true;

                    for (uint8_t i = 0; i < 4; i++)
                    {
                        bool received_after_disable =
                            can1.dm_rx_count[i] > dm_disable_rx_snapshot[i];

                        if (!received_after_disable ||
                            !DMmotor[i].hold_position_captured)
                        {
                            all_ready = false;
                        }
                        else if (DMmotor[i].error != 0)
                        {
                            dm_init_state = DM_FAULT;
                            all_ready = false;
                            break;
                        }
                    }

                    if (dm_init_state == DM_FAULT)
                    {
                        break;
                    }

                    if (all_ready)
                    {
                        dm_init_index = 0;
                        dm_init_state = DM_SEND_HOLD_TARGET;
                    }
                    else
                    {
                        DMmotor[dm_init_index].CanComm_ControlCmd(
                            can1,
                            CMD_RESET_MODE,
                            MOTOR_MODE + DMmotor[dm_init_index].ID
                        );

                        dm_init_index++;

                        if (dm_init_index >= 4)
                        {
                            dm_init_index = 0;
                        }
                    }
                    break;
                }

                case DM_SEND_HOLD_TARGET:
                    DMmotor[dm_init_index].DMmotor_transmit(DMmotor[dm_init_index].ID);

                    dm_init_index++;

                    if (dm_init_index >= 4)
                    {
                        dm_init_index = 0;
                        dm_init_state = DM_ENABLE_ALL;
                    }
                    break;

                case DM_ENABLE_ALL:
                    if (!DMmotor[dm_init_index].hold_position_captured ||
                        DMmotor[dm_init_index].error != 0)
                    {
                        dm_init_state = DM_FAULT;
                        break;
                    }

                    dm_enable_rx_snapshot[dm_init_index] =
                        can1.dm_rx_count[dm_init_index];

                    DMmotor[dm_init_index].CanComm_ControlCmd(
                        can1,
                        CMD_MOTOR_MODE,
                        MOTOR_MODE + DMmotor[dm_init_index].ID
                    );

                    dm_init_index++;

                    if (dm_init_index >= 4)
                    {
                        dm_init_index = 0;
                        dm_init_state = DM_WAIT_ENABLE;
                    }
                    break;
                case DM_WAIT_ENABLE:
                {
                    bool all_enabled = true;

                    for (uint8_t i = 0; i < 4; i++)
                    {
                        bool received_after_enable =
                            can1.dm_rx_count[i] > dm_enable_rx_snapshot[i];

                        if (!received_after_enable ||
                            DMmotor[i].error != 1)
                        {
                            all_enabled = false;
                        }

                        if (received_after_enable &&
                            DMmotor[i].error > 1)
                        {
                            dm_init_state = DM_FAULT;
                            all_enabled = false;
                            break;
                        }
                    }

                    if (dm_init_state == DM_FAULT)
                    {
                        break;
                    }

                    if (all_enabled)
                    {
                        ctrl.init_DM = 1;
                        dm_init_index = 0;
                        dm_init_state = DM_READY;
                    }
                    else
                    {
                        bool current_motor_enabled =
                            can1.dm_rx_count[dm_init_index]
                > dm_enable_rx_snapshot[dm_init_index]
                            && DMmotor[dm_init_index].error == 1;

                        if (!current_motor_enabled)
                        {
                            DMmotor[dm_init_index].CanComm_ControlCmd(
                                can1,
                                CMD_MOTOR_MODE,
                                MOTOR_MODE + DMmotor[dm_init_index].ID
                            );
                        }

                        dm_init_index++;

                        if (dm_init_index >= 4)
                        {
                            dm_init_index = 0;
                        }
                    }

                    break;
                }
                case DM_READY:
                    break;

                case DM_FAULT:
                    break;

                default:
                    break;
                }
            }

            for (auto& dm : DMmotor)
            {
                dm.DMmotor_transmit(dm.ID);
            }
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8);
			can2.Transmit(0x1ff, can2.temp_data + 8);
			break;
		case 2:
			can1.Transmit(0x200, can1.temp_data);
			can2.Transmit(0x200, can2.temp_data);
		default:
			break;
		}
		
		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//开始执行该任务之后1ms再执行该任务

	}
}

void ControlTask(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        /*
         * 先更新遥控器和模式。
         * ROTATION模式下会把speedz设为1000。
         */
        rc.Update();

        Motor* yaw_motor =
            ctrl.pantile_motor[CONTROL::PANTILE::YAW];

        Motor* pitch_motor =
            ctrl.pantile_motor[CONTROL::PANTILE::PITCH];

        static CONTROL::MODE previous_mode = CONTROL::RESET;

        if (ctrl.mode != previous_mode)
        {
            // 锁存当前Yaw世界角度和机械角度
            ctrl.pantile.set_yaw =
                imu_pantile.GetAngleYaw();

            ctrl.pantile.mark_yaw =
                yaw_motor->angle[now];

            ctrl.pantile.base_mark_yaw =
                ctrl.pantile.mark_yaw;

            // 锁存当前Pitch连续机械角度
            if (pitch_motor->continuous_initialized)
            {
                ctrl.pantile.mark_pitch =
                    static_cast<float>(pitch_motor->sum_angle);

                ctrl.pantile.base_mark_pitch =
                    ctrl.pantile.mark_pitch;
            }

            // 清除上一模式的Yaw前馈
            ctrl.pantile.yaw_speed_ff = 0.0f;
            ctrl.pantile.yaw_speed_ff_target = 0.0f;
            yaw_motor->speed_feedforward = 0.0f;

            ctrl.pantile.yaw_hold_initialized =
                ctrl.mode != CONTROL::RESET;
        }
        const bool xuc_tracking =
            xuc.track_flag &&
            ctrl.mode != CONTROL::RESET;

        if (ctrl.mode == CONTROL::RESET)
        {
            ctrl.pantile.yaw_hold_initialized = false;

            /*
             * RESET时关闭所有Yaw速度前馈。
             */
            ctrl.pantile.yaw_speed_ff = 0.0f;
            ctrl.pantile.yaw_speed_ff_target = 0.0f;
            yaw_motor->speed_feedforward = 0.0f;
        }
        else
        {
            if (!ctrl.pantile.yaw_hold_initialized)
            {
                /*
                 * 记录进入运行状态时的世界Yaw方向。
                 */
                ctrl.pantile.set_yaw = imu_pantile.GetAngleYaw();

                ctrl.pantile.mark_yaw = yaw_motor->angle[now];

                ctrl.pantile.base_mark_yaw = ctrl.pantile.mark_yaw;

                /*
                 * 模式切换时先清空前馈，
                 * 防止继承上一次小陀螺输出。
                 */
                ctrl.pantile.yaw_speed_ff = 0.0f;
                ctrl.pantile.yaw_speed_ff_target = 0.0f;
                yaw_motor->speed_feedforward = 0.0f;

                ctrl.pantile.yaw_hold_initialized = true;
            }

            if (ctrl.mode == CONTROL::MANUAL_YAW && !xuc_tracking)
            {
                constexpr float manual_yaw_max_rate_degree = 240.0f;
                constexpr float control_period = 0.005f;
                constexpr float encoder_per_degree =
                    8192.0f / 360.0f;

                /*
                 * 测试位置环时，摇杆直接修改机械编码器目标。
                 * 摇杆归中后 mark_yaw 保持不变。
                 */
                if (rc.rc.ch[2] > 100 ||
                    rc.rc.ch[2] < -100)
                {
                    ctrl.pantile.mark_yaw +=
                        static_cast<float>(rc.rc.ch[2])
                        / 660.0f
                        * manual_yaw_max_rate_degree
                        * control_period
                        * encoder_per_degree;
                }

                /*
                 * 右摇杆竖向 ch[3] 控制 Pitch。
                 */
                if (rc.rc.ch[3] > 100 ||
                    rc.rc.ch[3] < -100)
                {
                    ctrl.Control_Pantile(
                        0,
                        rc.rc.ch[3]
                        * para.pitch_speed
                        / 660.0f
                    );
                }
            }



            /*
             * 1. 底盘旋转指令前馈
             *
             * ROTATION下speedz=1000。
             * 在底盘真正转动、世界角误差出现之前，
             * 就提前命令Yaw电机向反方向旋转。
             */
            float command_feedforward = 0.0f;
            float gyro_feedback = 0.0f;

            if (ctrl.mode == CONTROL::ROTATION ||
                ctrl.mode == CONTROL::FOLLOW)
            {
                command_feedforward =
                    ctrl.pantile.yaw_cmd_ff_k *
                    ctrl.chassis.speedz;

                gyro_feedback =
                    ctrl.pantile.yaw_gyro_fb_k *
                    imu_pantile.GetAngularVelocityYaw();
            }

            ctrl.pantile.yaw_speed_ff_target =
                command_feedforward + gyro_feedback;

            /*
             * 前馈限幅。
             * 初期建议限制在±300以内。
             */
            if (ctrl.pantile.yaw_speed_ff_target > ctrl.pantile.yaw_ff_limit)
            {
                ctrl.pantile.yaw_speed_ff_target = ctrl.pantile.yaw_ff_limit;
            }
            else if (ctrl.pantile.yaw_speed_ff_target < -ctrl.pantile.yaw_ff_limit)
            {
                ctrl.pantile.yaw_speed_ff_target = -ctrl.pantile.yaw_ff_limit;
            }

            /*
             * 一阶低通，避免模式切换时目标速度瞬间跳变。
             *
             * ControlTask周期5ms，系数0.25时响应仍然很快。
             */
            ctrl.pantile.yaw_speed_ff += ctrl.pantile.yaw_ff_filter * (ctrl.pantile.yaw_speed_ff_target - ctrl.pantile.yaw_speed_ff);

            /*
             * 将速度前馈送给原有POS位置—速度双环。
             */
            yaw_motor->speed_feedforward =
                ctrl.pantile.yaw_speed_ff;
            if (xuc_tracking)
            {
                ctrl.pantile.set_yaw =
                    ctrl.GetDelta(xuc.yaw);

                ctrl.pantile.Keep_Pantile(
                    xuc.pitch * 180.0f / PI,
                    CONTROL::PANTILE::PITCH,
                    imu_pantile
                );

                ctrl.pantile.Keep_Pantile(
                    ctrl.pantile.set_yaw,
                    CONTROL::PANTILE::YAW,
                    imu_pantile
                );
            }
            
            /*
             * 原世界角度保持逻辑不变。
             */
            else if (ctrl.mode == CONTROL::MANUAL_YAW)
            {
                // 纯机械位置环测试时关闭世界角速度前馈
                ctrl.pantile.yaw_speed_ff = 0.0f;
                ctrl.pantile.yaw_speed_ff_target = 0.0f;
                yaw_motor->speed_feedforward = 0.0f;
            }
            else
            {
                ctrl.pantile.Keep_Pantile(
                    ctrl.pantile.set_yaw,
                    CONTROL::PANTILE::YAW,
                    imu_pantile
                );
            }
        }

        ctrl.pantile.Update();
        ctrl.chassis.Update();
        ctrl.shooter.Update();
        xuc.Encode();

        previous_mode = ctrl.mode;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}


void DecodeTask(void* pvParameters)
{
	while (true)
	{
		rc.Decode();

		imu_pantile.Decode();
		xuc.Decode();
	
		vTaskDelay(5);
	}
}

void ArmTask(void* pvParameters)
{
	while (true)
	{
		//初始化达妙电机
	//	DMmotor[0].DMmotorinit();
	//	power.Send();
	//	vTaskDelay(100);
	}
}





