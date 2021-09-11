#ifndef __ENCODER_H_
#define __ENCODER_H_
#include "stm32f4xx.h"

/*����״̬ö��*/
typedef enum
{
		NORM,   //����
		BLOCK,  //��ת
		WRONG 	//�쳣
}STATE_e;

/*CAN���ݴ���-���̴���*/
typedef __packed struct
{
		u8 						Ahead[2];
		u8 						Back[2];
		int16_t 			Radio_Circle;
		int16_t 			Actual_Circle;
	  int32_t 			Encode_Record_Val;
		int32_t 			Encode_Actual_Val;

		float 				Lock_Radian;
		STATE_e 			State;
		float 				Radian;
		float 				Init_Radian;
		float         Total_Radian;
	
		int16_t				Speed[2];
		int16_t				AccSpeed;
}Encoder_t;


/*CAN��������ֵ����*/
void CAN_DATA_Encoder_Deal(u8 radio,int16_t CanData,int16_t speed,Encoder_t * Encoder);

/*�����ٶȴ���*/
void CAN_DATA_Speed_Deal(int16_t speed,Encoder_t * Encoder);

/*����ֵ��ֵ���㴦��*/
void EncoderValZero(Encoder_t * Encoder);

#endif
