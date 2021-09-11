/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1���պͷ��͵������
  ******************************************************************************
  */

#include "can_1_receive.h"
#include "rc.h"
#include "rm_motor.h"
#include "chassis_task.h"

//������ݶ�ȡ
#define get_motor_M3508(ptr, rx_message)                                                  \
    {                                                                                     \
        (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
        (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);    \
    }

/*--------------------����-----------------------*/
//�������̵������ static
motor_measure_t motor_chassis[4];
//���������������
motor_measure_t motor_fire;
//����С��̨�������
motor_measure_t yaw_6020;
motor_measure_t pitch_2006;		
//������Ԯ�������	
motor_measure_t rescue_motor;
motor_measure_t rescue_clap;
////����yaw��3508�������
//static motor_measure_t motor_yaw;
Supercapacitor_receive_t Supercap_receive;

//��������̨�������
//static motor_measure_t motor_second_yaw;
//����pitch��������
static motor_measure_t motor_pitch;

/*--------------------����-----------------------*/
void (*can1_callback)(CanRxMsg *);

const Supercapacitor_receive_t *get_supercap_control_point(void)
{
    return &Supercap_receive;
}
//���ز������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_fire_motor_measure_point(void)
{
    return &motor_fire;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_pitch;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

//����С��̨���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_spitch_motor_measure_point(void)
{
    return &pitch_2006;
}

const motor_measure_t *get_syaw_motor_measure_point(void)
{
    return &yaw_6020;
}

//���ؾ�Ԯ���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_rescue_motor_measure_point(void)
{
    return &rescue_motor;
}

const motor_measure_t *get_rescue_clap_motor_measure_point(void)
{
    return &rescue_clap;
}

/*=======================��̨���==============================*/
motor_measure_t uplift;
motor_measure_t	translation;
motor_measure_t	telescoping;
motor_measure_t	clip;
motor_measure_t	flip;

//������̨���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
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





//can1�����ж�
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
/*************************************can1����*************************************/
/**********************************************************************************/
void chassis_can1_callback(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
    /*���̵��*/
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
    /*�������*/
    case 0x207:
    {
        get_motor_M3508(&motor_fire, rx_message);
        break;
    }
#endif
    case 0x211: //�������ݽ���
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
    /*Pitch�������λ�÷���*/
    case 0x01:
    {
        motor_pitch.position = (rx_message->Data[6] << 24 | rx_message->Data[5] << 16 | rx_message->Data[4] << 8 | rx_message->Data[3]);
#if (INFANTRY_NUM == INFANTRY_4)
        motor_pitch.actual_Position = -(motor_pitch.position - Pitch_Middle_Angle); //ʵ��ֵ��ȥ�м�ֵ
#elif (INFANTRY_NUM == INFANTRY_3)
        motor_pitch.actual_Position = -(motor_pitch.position - Pitch_Middle_Angle); //ʵ��ֵ��ȥ�м�ֵ
#endif
        break;
    }

    /*P��3510����ٶȷ���*/
    case 0x5555:
    {
        motor_pitch.speed = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
        motor_pitch.given_current = (uint16_t)(rx_message->Data[4] << 8 | rx_message->Data[5]);

        break;
    }

#ifdef DOUBLE_WORK
    /*����̨yaw��2006������ݷ���*/
    case 0x202:
    {
        motor_second_yaw.position = (rx_message->Data[0] << 8) | rx_message->Data[1];
        Motor_Actual_Position(&motor_second_yaw, Sec_YAW_RATIO, 8192);
        motor_second_yaw.speed = (rx_message->Data[2] << 8) | rx_message->Data[3];
        break;
    }
#endif
		
		
/*=======================��̨���==============================*/
		
			/*̧������ٶȷ���*/
			case 0x201:
			{
					get_motor_M3508(&uplift, rx_message);
					motor_actual_position(&uplift,LIFT_RATIO,8192);
					break;
			}
			/*ƽ�Ƶ���ٶȷ���*/
			case 0x202:
			{
					get_motor_M3508(&translation, rx_message);
					motor_actual_position(&translation,RESCUE_RATIO,8192);
					break;
			}
			/*��������ٶȷ���*/
			case 0x203:
			{
					get_motor_M3508(&telescoping, rx_message);
					motor_actual_position(&telescoping,FUNCTION_RATIO,8192);
					break;
			}
			/*�н�����ٶȷ���*/
			case 0x204:
			{
					get_motor_M3508(&clip, rx_message);
					motor_actual_position(&clip,FUNCTION_RATIO,8192);
					break;
			}
			/*��ת����ٶȷ���*/
			case 0x205:
			{
					get_motor_M3508(&flip, rx_message);
					motor_actual_position(&flip,FUNCTION_RATIO,8192);
					
					break;
			}
    }
}

//�������ݵ�ѹ
float re_capacitance_voltage(void)
{
    return (Supercap_receive.Capacitance_voltage);
}
