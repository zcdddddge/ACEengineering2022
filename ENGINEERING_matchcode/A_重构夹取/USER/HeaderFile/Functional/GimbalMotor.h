#ifndef __GIMBALMOTOR_H_
#define __GIMBALMOTOR_H_
#include "motor.h"

/*��̨�ṹ��*/
typedef __packed struct
{
	Motor_t PitchMotor;
	Motor_t YawMotor;
	Motor_t	AmmunitiMotor;
	int16_t *GimbalData;
	void (*Can_Send_Gimbal)(int16_t,int16_t,int16_t,int16_t);
	int16_t* (*Get_Gimbal_Data)(void);
	Encoder_t*(*Get_Pitch_Encoder)(void);
	Encoder_t*(*Get_Yaw_Encoder)(void);
}G_t;



/*PY������ʼ��*/
void PY_Motor_Init(G_t *G);


/*��ȡ��̨�������*/
void Get_Gimbal_Data(G_t *G);


/*PY�����̿���ģʽ*/
void PY_Encoder_DRIVE(G_t *G,float P,float Y,float Y_SPEED);


/*��̨�ϵ�*/
void Gimbal_Poweroff(G_t *G);


/*���������ʼ��*/
void Ammuniti_Motor_Init(G_t *G);


/*�����������*/
void Ammuniti_Ctrl(G_t *G,int8_t SpeedGain);

#endif
