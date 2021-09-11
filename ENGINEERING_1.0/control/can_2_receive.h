/**
  ******************************************************************************
  * @file       CAN_2_Receive.c/h
  * @brief      通过CAN2进行板间通讯
  ******************************************************************************
  */

#ifndef __CAN_2_RECEIVE_H__
#define __CAN_2_RECEIVE_H__
#include "struct_typedef.h"
#include "rc.h"
#include "can_1_receive.h"


typedef struct //__packed
{		
	int8_t photoelectric_zero;  //Y轴中值光电标志
	int8_t Gimbal_supply_flag;//补给状态标志位(0:归中状态，进入补给状态  1:云台90度转动中  2:到达指定位置  3:归中中   4：补给模式结束)
	int8_t Gimbal_all_flag;//发送给底盘的初始化成功标志
	
	float chassis_gimbal_angel;
	int16_t pitch_angle;
	uint8_t gimbal_beh;
	
	u8 SensorL;
	u8 SensorR;
	
} gimbal_yaw_receive_t;



//返回接收云台数据，通过指针方式获取原始数据
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
