#ifndef __GIMBALMOTOR_H_
#define __GIMBALMOTOR_H_
#include "motor.h"
#include "RemoteDeal.h"


/*��̨�ṹ��*/
typedef __packed struct
{
	Motor_t PitchMotor;
	Motor_t YawMotor;
	REMOTE_t *Rc ;
	void (*Can_Send_Gimbal)(int16_t,int16_t);
	void (*Can_Send_Y)(int16_t );
}G_t;





/*PY������ʼ��*/
void PY_Motor_Init(G_t *G);


/*PY�����̿���ģʽ*/
void PY_Encoder_DRIVE(G_t *G,float P,float Y,float Y_SPEED);

void PY_Motor(G_t *G );

/*��̨�ϵ�*/
void Gimbal_Poweroff(G_t *G);


/*���������ʼ��*/
void Ammuniti_Motor_Init(G_t *G);


/*�����������*/
void Ammuniti_Ctrl(G_t *G,int8_t SpeedGain);

#endif
