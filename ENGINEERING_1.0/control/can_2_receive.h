/**
  ******************************************************************************
  * @file       CAN_2_Receive.c/h
  * @brief      ͨ��CAN2���а��ͨѶ
  ******************************************************************************
  */

#ifndef __CAN_2_RECEIVE_H__
#define __CAN_2_RECEIVE_H__
#include "struct_typedef.h"
#include "rc.h"
#include "can_1_receive.h"


typedef struct //__packed
{		
	int8_t photoelectric_zero;  //Y����ֵ����־
	int8_t Gimbal_supply_flag;//����״̬��־λ(0:����״̬�����벹��״̬  1:��̨90��ת����  2:����ָ��λ��  3:������   4������ģʽ����)
	int8_t Gimbal_all_flag;//���͸����̵ĳ�ʼ���ɹ���־
	
	float chassis_gimbal_angel;
	int16_t pitch_angle;
	uint8_t gimbal_beh;
	
	u8 SensorL;
	u8 SensorR;
	
} gimbal_yaw_receive_t;



//���ؽ�����̨���ݣ�ͨ��ָ�뷽ʽ��ȡԭʼ����
const gimbal_yaw_receive_t *get_yaw_receive_measure_point(void);
motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
uint32_t gimbal_rc_lost_time(void);

float re_can2_shooter_heat0_speed_limit(void);
float re_chassis_gimbal_angel(void);
u8 re_chassis_gimbal_sensorL(void);
u8 re_chassis_gimbal_sensorR(void);
float re_gimbal_pitch_angle(void);
int re_gimbal_behaviour(void);

extern void (*can2_callback)(CanRxMsg *);
void chassis_can2_callback(CanRxMsg *rx_message);
void gimbal_can2_callback(CanRxMsg *rx_message);

#endif
