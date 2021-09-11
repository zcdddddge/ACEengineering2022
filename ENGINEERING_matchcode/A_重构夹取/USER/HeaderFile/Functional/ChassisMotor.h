#ifndef __CHASSISMOTOR_H_
#define __CHASSISMOTOR_H_
#include "motor.h"



/*���̽ṹ��*/
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



/*�������ӳ�ʼ��*/
void Wheel_Motor_Init(C_t *C);


/*���̸������*/
void Chassis_Text_Drive(C_t *C);


/*���̸���*/
void Chassis_Follow_Drive(C_t *C,float X_IN,float Y_IN,float Z_IN);


/*���̶ϵ�*/
void Chassis_Poweroff_Drive(C_t *C);


/*��Ԯ�����ʼ��*/
void Rescue_Motor_Init(C_t *C);


/*������Ԯ���*/
void Rescue_Ctrl(C_t *C,int8_t choice);
	

/*��ȡ���̵������*/
void Get_Chassis_Data(C_t *C);

#endif
