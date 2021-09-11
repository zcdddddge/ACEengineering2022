#include "can_2_receive.h"
#include "rc.h"
#include "iwdg.h"
#include "rm_motor.h"
#include "referee_deal.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "parameter.h"

/*--------------------变量-----------------------*/
motor_measure_t motor_yaw;
extern rc_ctrl_t rc_ctrl;
static gimbal_yaw_receive_t gimbal_receive;

QueueHandle_t CAN2_Handle = NULL;
static portTickType CAN2LostTime = 0;
static uint8_t shooter_heat0_speed_limit = 0;

/*--------------------函数-----------------------*/
void (*can2_callback)(CanRxMsg *);

//返回接收云台数据，通过指针方式获取原始数据
const gimbal_yaw_receive_t *get_yaw_receive_measure_point(void)
{
	//gimbal_receive.Gimbal_all_flag = 1;
    return &gimbal_receive;
}
//返回yaw电机变量地址，通过指针方式获取原始数据
motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_yaw;
}

uint32_t gimbal_rc_lost_time(void)
{
    return CAN2LostTime;
}

static void can2_lost_time_refresh(void)
{
    /* 当接收到数据时，刷新失联倒计时   xTaskGetTickCountFromISR  xTaskGetTickCount  */
    CAN2LostTime = xTaskGetTickCount() + ((uint32_t)50);
    iwdg_feed();
}

//can2接收中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);

        if (can2_callback != NULL)
            can2_callback(&rx2_message);
    }
}

/**********************************************************************************/
/*************************************can2接收*************************************/
/**********************************************************************************/
void chassis_can2_callback(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
				case 0x300:
				{
						gimbal_receive.Gimbal_all_flag = rx_message->Data[0] / 100;
				//gimbal_receive.Gimbal_all_flag = 1;
						gimbal_receive.photoelectric_zero = !((rx_message->Data[0] % 100) / 10);
						gimbal_receive.Gimbal_supply_flag = (rx_message->Data[0]) % 10;

						gimbal_receive.chassis_gimbal_angel = (((rx_message)->Data[1] << 8 | (rx_message)->Data[2]) / 100.f) - 180.f;
						gimbal_receive.gimbal_beh = (rx_message)->Data[3];
						gimbal_receive.pitch_angle = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
						break;
				}
				case 0x301:
				{
						gimbal_receive.SensorL = rx_message->Data[0];
						gimbal_receive.SensorR = rx_message->Data[1];
					
						break;
				}
    }
}

void gimbal_can2_callback(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    //can2_lost_time_refresh();
    /*yaw轴3508电机速度*/
    case 0x205:
    {
        motor_yaw.position = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
        motor_actual_position(&motor_yaw, YAW_RATIO, 8192); //计算yaw电机的真实码盘值
        motor_yaw.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];
        break;
    }

    case 0x400:
    {
        can2_lost_time_refresh();
        rc_ctrl.rc.ch[2] = ((rx_message->Data[0] << 8) | rx_message->Data[1]);
        rc_ctrl.rc.ch[3] = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
        rc_ctrl.rc.ch[4] = (rx_message->Data[4] << 8 | rx_message->Data[5]);
        rc_ctrl.rc.s1 = (rx_message->Data[6]) / 10;
        rc_ctrl.rc.s2 = (rx_message->Data[6]) % 10;
        rc_ctrl.mouse.press_l = (rx_message->Data[7]) / 10 - 1;
        rc_ctrl.mouse.press_r = (rx_message->Data[7]) % 10;

        break;
    }
    case 0x401:
    {
        can2_lost_time_refresh();
        rc_ctrl.mouse.x = ((rx_message->Data[0] << 8) | rx_message->Data[1]);
        rc_ctrl.mouse.y = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
        rc_ctrl.kb.key_code = (rx_message->Data[4] << 8 | rx_message->Data[5]);
        rc_ctrl.rc.s1 = (rx_message->Data[6]) / 10;
        rc_ctrl.rc.s2 = (rx_message->Data[6]) % 10;
        rc_ctrl.mouse.press_l = (rx_message->Data[7]) / 10 - 1;
        rc_ctrl.mouse.press_r = (rx_message->Data[7]) % 10;

        break;
    }
    case 0x402:
    {
        shooter_heat0_speed_limit = ((rx_message->Data[0] << 8) | rx_message->Data[1]);

        break;
    }
    }
}

/**
  * @brief          返回can2接收回来的17mm枪管射速上限
  * @param[in]      void
  * @retval         17mm枪管射速上限
  * @attention      用于云台板
  */
float re_can2_shooter_heat0_speed_limit(void)
{
    return (shooter_heat0_speed_limit);
}

/**
  * @brief          返回can2接收回来的底盘和云台的差角
  * @param[in]      void
  * @retval         底盘和云台的差角
  * @attention      用于底盘板
  */
float re_chassis_gimbal_angel(void)
{
    return (gimbal_receive.chassis_gimbal_angel);
}

/**
  * @brief          返回can2接收回来的传感器数据
  * @param[in]      void
  * @retval         底盘自动
  * @attention      用于底盘板
  */
u8 re_chassis_gimbal_sensorL(void)
{
    return (gimbal_receive.SensorL);
}

/**
  * @brief          返回can2接收回来的传感器数据
  * @param[in]      void
  * @retval         底盘自动
  * @attention      用于底盘板
  */
u8 re_chassis_gimbal_sensorR(void)
{
    return (gimbal_receive.SensorR);
}



/**
  * @brief          返回can2接收回来的pitch轴角度
  * @param[in]      void
  * @retval         pitch轴角度
  * @attention      用于底盘板
  */
float re_gimbal_pitch_angle(void)
{
    return (fp32)(gimbal_receive.pitch_angle * 360.0f / 1024);
}

/**
  * @brief          返回can2接收回来的云台模式
  * @param[in]      void
  * @retval         云台模式
  * @attention      用于底盘板
  */
int re_gimbal_behaviour(void)
{
    return (int)(gimbal_receive.gimbal_beh);
}
