/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1接收和发送电机数据
  ******************************************************************************
  */

#include "can_1_receive.h"
#include "rc.h"
#include "rm_motor.h"
#include "chassis_task.h"

//电机数据读取
#define get_motor_M3508(ptr, rx_message)                                                  \
    {                                                                                     \
        (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
        (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);    \
    }

/*--------------------变量-----------------------*/
//申明底盘电机变量 static
motor_measure_t motor_chassis[4];
//申明拨弹电机变量
motor_measure_t motor_fire;
//申明小云台电机变量
motor_measure_t yaw_6020;
motor_measure_t pitch_2006;		
//申明救援电机变量	
motor_measure_t rescue_motor;
motor_measure_t rescue_clap;
////申明yaw轴3508电机变量
//static motor_measure_t motor_yaw;
Supercapacitor_receive_t Supercap_receive;

//申明副云台电机变量
//static motor_measure_t motor_second_yaw;
//申明pitch轴电机变量
static motor_measure_t motor_pitch;

/*--------------------函数-----------------------*/
void (*can1_callback)(CanRxMsg *);

const Supercapacitor_receive_t *get_supercap_control_point(void)
{
    return &Supercap_receive;
}
//返回拨弹电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_fire_motor_measure_point(void)
{
    return &motor_fire;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_pitch;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

//返回小云台电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_spitch_motor_measure_point(void)
{
    return &pitch_2006;
}

const motor_measure_t *get_syaw_motor_measure_point(void)
{
    return &yaw_6020;
}

//返回救援电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_rescue_motor_measure_point(void)
{
    return &rescue_motor;
}

const motor_measure_t *get_rescue_clap_motor_measure_point(void)
{
    return &rescue_clap;
}

/*=======================云台电机==============================*/
motor_measure_t uplift;
motor_measure_t	translation;
motor_measure_t	telescoping;
motor_measure_t	clip;
motor_measure_t	flip;

//返回云台电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_uplift_measure_point(void)
{
    return &uplift;
}

const motor_measure_t *get_translation_measure_point(void)
{
    return &translation;
}

const motor_measure_t *get_telescoping_measure_point(void)
{
    return &telescoping;
}

const motor_measure_t *get_clip_measure_point(void)
{
    return &clip;
}

const motor_measure_t *get_flip_measure_point(void)
{
    return &flip;
}





//can1接收中断
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx1_message;
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);

        if (can1_callback != NULL)
            can1_callback(&rx1_message);
    }
}

/**********************************************************************************/
/*************************************can1接收*************************************/
/**********************************************************************************/
void chassis_can1_callback(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    /*底盘电机*/
    case 0x201:
    {
        get_motor_M3508(&motor_chassis[0], rx_message);
        break;
    }
    case 0x202:
    {
        get_motor_M3508(&motor_chassis[1], rx_message);
        break;
    }
    case 0x203:
    {
        get_motor_M3508(&motor_chassis[2], rx_message);
        break;
    }
    case 0x204:
    {
        get_motor_M3508(&motor_chassis[3], rx_message);
        break;
    }
    case 0x205:
    {
        get_motor_M3508(&rescue_motor, rx_message);
				motor_actual_position(&rescue_motor,RESCUE_RATIO,8192);
				
        break;
    }
    case 0x206:
    {
        get_motor_M3508(&rescue_clap, rx_message);
				motor_actual_position(&rescue_clap,RESCUE_RATIO,8192);
        break;
    }
    case 0x20B:
    {
        get_motor_M3508(&yaw_6020, rx_message);
				motor_actual_position(&yaw_6020,SYAW_RATIO,8192);
        break;
    }
    case 0x208:
    {
        get_motor_M3508(&pitch_2006, rx_message);
				motor_actual_position(&pitch_2006,SPITCH_RATIO,8192);
        break;
    }
		
		
#ifdef FIRE_WORK
    /*拨弹电机*/
    case 0x207:
    {
        get_motor_M3508(&motor_fire, rx_message);
        break;
    }
#endif
    case 0x211: //超级电容接收
    {
        uint16_t *cap_redata = (uint16_t *)rx_message->Data;
        Supercap_receive.input_voltage = (float)(cap_redata[0] / 100.0f);
        Supercap_receive.Capacitance_voltage = (float)(cap_redata[1] / 100.0f);
        Supercap_receive.Input_current = (float)(cap_redata[2] / 100.0f);
        Supercap_receive.Set_power = (float)(cap_redata[3] / 100.0f);
        break;
    }

    default:
        break;
    }
}

void gimbal_can1_callback(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    /*Pitch轴编码器位置反馈*/
    case 0x01:
    {
        motor_pitch.position = (rx_message->Data[6] << 24 | rx_message->Data[5] << 16 | rx_message->Data[4] << 8 | rx_message->Data[3]);
#if (INFANTRY_NUM == INFANTRY_4)
        motor_pitch.actual_Position = -(motor_pitch.position - Pitch_Middle_Angle); //实际值减去中间值
#elif (INFANTRY_NUM == INFANTRY_3)
        motor_pitch.actual_Position = -(motor_pitch.position - Pitch_Middle_Angle); //实际值减去中间值
#endif
        break;
    }

    /*P轴3510电机速度反馈*/
    case 0x5555:
    {
        motor_pitch.speed = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
        motor_pitch.given_current = (uint16_t)(rx_message->Data[4] << 8 | rx_message->Data[5]);

        break;
    }

#ifdef DOUBLE_WORK
    /*副云台yaw轴2006电机数据反馈*/
    case 0x202:
    {
        motor_second_yaw.position = (rx_message->Data[0] << 8) | rx_message->Data[1];
        Motor_Actual_Position(&motor_second_yaw, Sec_YAW_RATIO, 8192);
        motor_second_yaw.speed = (rx_message->Data[2] << 8) | rx_message->Data[3];
        break;
    }
#endif
		
		
/*=======================云台电机==============================*/
		
			/*抬升电机速度反馈*/
			case 0x201:
			{
					get_motor_M3508(&uplift, rx_message);
					motor_actual_position(&uplift,LIFT_RATIO,8192);
					break;
			}
			/*平移电机速度反馈*/
			case 0x202:
			{
					get_motor_M3508(&translation, rx_message);
					motor_actual_position(&translation,RESCUE_RATIO,8192);
					break;
			}
			/*伸缩电机速度反馈*/
			case 0x203:
			{
					get_motor_M3508(&telescoping, rx_message);
					motor_actual_position(&telescoping,FUNCTION_RATIO,8192);
					break;
			}
			/*夹紧电机速度反馈*/
			case 0x204:
			{
					get_motor_M3508(&clip, rx_message);
					motor_actual_position(&clip,FUNCTION_RATIO,8192);
					break;
			}
			/*翻转电机速度反馈*/
			case 0x205:
			{
					get_motor_M3508(&flip, rx_message);
					motor_actual_position(&flip,FUNCTION_RATIO,8192);
					
					break;
			}
    }
}

//超级电容电压
float re_capacitance_voltage(void)
{
    return (Supercap_receive.Capacitance_voltage);
}
