#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "stm32f4xx.h"
#include "CAN1_ISR.h"
#include "CAN2_ISR.h"
#include "pid.h"

/*�������ö��*/
typedef enum
{
		CHASSIS_M,		 //����
		PITCH_M,			 //Pitch
		YAW_M,  			 //Yaw
		SWING_M,			 //��ת
		CLAMP_M,			 //��ȡ
		TRANS_M,			 //����
		SMOOTH_M,			 //����
		SLIDE_M,		 	 //̧��
		RESCUE_M,      //��Ԯ
		SUPPLY_M,			 //����
		AMMUNITI_M,		 //����
		CURRENCY_M     //ͨ��
}MotorType_e;


/*����ṹ��*/
typedef __packed struct
{
	uint8_t ID;						//���ID��
	uint8_t Pos_Lock;			//λ����
	int16_t Radio;				//���ٱ�
	int32_t ExpSpeed;			//�����ٶ�
	float ExpRadian;			//�����Ƕ�
	PID_t SPID;						//�ٶȻ�PID
	PID_t PPID;						//λ�û�PID
	Encoder_t*Encoder;		//����
	MotorType_e MotorType;//�������
}Motor_t;


/*�����ֵ����*/
void MotorValZero(Motor_t *motor);

#endif
