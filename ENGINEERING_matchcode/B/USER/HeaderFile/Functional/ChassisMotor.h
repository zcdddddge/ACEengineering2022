#ifndef __CHASSISMOTOR_H_
#define __CHASSISMOTOR_H_
#include "motor.h"
#include "PYR_ISR.h"
#include "RemoteDeal.h"
#include "BoardCommuni.h"

/**********************************Can发送数值************************************************/
#define Wheel_Output	 C->WheelMotor[0].SPID.Out,C->WheelMotor[1].SPID.Out,C->WheelMotor[2].SPID.Out,C->WheelMotor[3].SPID.Out
//#define Wheel_Output	 0,0,0,0
#define Can_205_208_Out_Put C->WheelMotor[4].SPID.Out, C->WheelMotor[5].SPID.Out, 0, 0
#define Can_205_206_Out_Put C->WheelMotor[4].SPID.Out , C->WheelMotor[5].SPID.Out
//#define Wheel_Output	 0,0,0,0
/*********************************底盘电机pid**************************************************/
/*底盘轮子速度环参数*/
#define WHEEL_MOTOR1_P   	8.5f
#define WHEEL_MOTOR1_I   	0.0f
#define WHEEL_MOTOR1_D   0.02f
#define WHEEL_MOTOR2_P  	8.5f
#define	WHEEL_MOTOR2_I   	0.0f
#define WHEEL_MOTOR2_D   0.02f
#define WHEEL_MOTOR3_P   	8.5f
#define WHEEL_MOTOR3_I   	0.0f
#define WHEEL_MOTOR3_D   0.02f
#define WHEEL_MOTOR4_P   	8.5f
#define WHEEL_MOTOR4_I   	0.0f
#define WHEEL_MOTOR4_D   0.02f
/*底盘救援电机参数*/
#define  RESCUE_S_P   10.0f
#define  RESCUE_S_I   0.0f
#define  RESCUE_S_D   1.0f
#define  RESCUE_P_P   10.0f
#define  RESCUE_P_I   0.0f
#define  RESCUE_P_D   0.0f

/**底盘救援爪电机**/ 
#define  BARRIER_S_P   100.0f
#define  BARRIER_S_I   0.0f
#define  BARRIER_S_D   1.0f
#define  BARRIER_P_P   10.0f
#define  BARRIER_P_I   0.0f
#define  BARRIER_P_D   1.0f 



/*Yaw闭环pid*/
#define Yaw_P					40.0f
#define Yaw_I					0.0f
#define Yaw_D					10.0f
/**************************************************速度定义**********************************/
static const int16_t Rescue_Speed    = 3000;   //救援电机速度
static const int16_t Barrier_Speed   = 2040; 

/*底盘结构体*/
typedef __packed struct
{
	Motor_t WheelMotor[6];
	
	PYR_t *gyro;
	PID_t Yaw_Pid;
	void (*Can_Send_Wheel)(int16_t,int16_t,int16_t,int16_t);
	void (*Can_Send_205_206)(int16_t,int16_t);
	void (*Can2_Send_205_208)(int16_t,int16_t,int16_t,int16_t);
	Encoder_t*(*Get_Encoder)(uint8_t  ch);
	Encoder_t*(*Get_206_Encoder)(void);
	PYR_t* (*Get_PYR_t)(void);
}C_t;



/*底盘轮子初始化*/
void Wheel_Motor_Init(C_t *C);


/*底盘测试*/
void Chassis_Text_Drive(C_t *C);


/*底盘独立模式*/
void Chassis_Indepen_Drive(C_t *C,float X_IN,float Y_IN,float Z_IN,int16_t ExpRescue);


/*底盘直走模式*/
void Chassis_Straight_Drive(C_t *C,int16_t speed);


/*底盘跟随模式*/
void Chassis_Follow_Drive(C_t *C,float X_IN,float Y_IN,float Z_IN);


/*底盘大陀螺模式*/
void Chassis_Rotation_Drive(C_t *C,float X_IN,float Y_IN,float Z_IN);


/*底盘扭腰模式*/
void Chassis_Wiggle_Drive(C_t *C,float X_IN,float Y_IN,float Z_IN);

/*底盘自动模式*/
void Chassis_Auto_Drive(C_t *C, float X_IN, float Y_IN, float Z_IN);

/*底盘断电*/
void Chassis_Poweroff_Drive(C_t *C);

/*驱动救援爪*/
void Chassis_Barrier(C_t *C, int16_t dire);

	
/*驱动救援电机*/
void Chassis_Rescue(C_t *C ,int16_t dire  );


#endif
