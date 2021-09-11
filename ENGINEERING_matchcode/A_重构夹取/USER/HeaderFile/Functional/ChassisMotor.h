#ifndef __CHASSISMOTOR_H_
#define __CHASSISMOTOR_H_
#include "motor.h"



/*底盘结构体*/
typedef __packed struct
{
	Motor_t WheelMotor[4];
	Motor_t	RescueMotor;
	int16_t *WheelData;
	int16_t *RescueData;
	void (*Can_Send_Wheel)(int16_t,int16_t,int16_t,int16_t);
	int16_t* (*Get_Wheel_Data)(void);
	void (*Can_Send_Rescue)(int16_t,int16_t,int16_t,int16_t);
	int16_t* (*Get_Rescue_Data)(void);
	Encoder_t*(*Get_Rescue_Encoder)(void);
}C_t;



/*底盘轮子初始化*/
void Wheel_Motor_Init(C_t *C);


/*底盘跟随测试*/
void Chassis_Text_Drive(C_t *C);


/*底盘跟随*/
void Chassis_Follow_Drive(C_t *C,float X_IN,float Y_IN,float Z_IN);


/*底盘断电*/
void Chassis_Poweroff_Drive(C_t *C);


/*救援电机初始化*/
void Rescue_Motor_Init(C_t *C);


/*驱动救援电机*/
void Rescue_Ctrl(C_t *C,int8_t choice);
	

/*获取底盘电机数据*/
void Get_Chassis_Data(C_t *C);

#endif
