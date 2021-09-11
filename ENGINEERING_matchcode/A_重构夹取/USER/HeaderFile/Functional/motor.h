/*
 * @Author: your name
 * @Date: 2021-01-09 15:32:31
 * @LastEditTime: 2021-01-13 18:00:57
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \PROJECTd:\RMware\A\USER\HeaderFile\Functional\motor.h
 */
#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "stm32f4xx.h"
#include "CAN1_ISR.h"
#include "CAN2_ISR.h"
#include "pid.h"



/********************************电机状态************************************************/
static const u8 Finish 		= 1;
static const u8 DisFinish = 0;

/*电机种类枚举*/
typedef enum
{
		CHASSIS_M,		 //底盘
		PITCH_M,			 //Pitch
		YAW_M,  			 //Yaw
		SWING_M,			 //翻转
		CLAMP_M,			 //夹取
		TRANS_M,			 //传送
		SMOOTH_M,			 //滑轨
		SLIDE_M,		 	 //抬升
		RESCUE_M,      //救援
		SUPPLY_M,			 //补给
		AMMUNITI_M,		 //拨弹
		CURRENCY_M     //通用
}MotorType_e;


/*电机结构体*/
typedef __packed struct
{

	uint8_t ID;						//电机ID号
	uint8_t Pos_Lock;			//位置锁
	int16_t Radio;				//减速比
	int32_t ExpSpeed;			//期望速度
	float 	ExpRadian;		//期望角度
	uint8_t state;				//状态
	uint8_t last_state;
	PID_t 	SPID;					//速度环PID
	PID_t 	PPID;					//位置环PID
	Encoder_t*Encoder;		//码盘
	MotorType_e MotorType;//电机种类
	uint8_t ResetFlag;
	uint8_t debug;
	
}Motor_t;


/*电机数值清零*/
void MotorValZero(Motor_t *motor);

#endif
