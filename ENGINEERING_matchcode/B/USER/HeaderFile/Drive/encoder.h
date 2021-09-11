#ifndef __ENCODER_H_
#define __ENCODER_H_
#include "stm32f4xx.h"

/*数据状态枚举*/
typedef enum
{
		NORM,   //正常
		BLOCK,  //堵转
		WRONG 	//异常
}STATE_e;

/*CAN数据处理-码盘处理*/
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


/*CAN返回码盘值处理*/
void CAN_DATA_Encoder_Deal(u8 radio,int16_t CanData,int16_t speed,Encoder_t * Encoder);

/*码盘速度处理*/
void CAN_DATA_Speed_Deal(int16_t speed,Encoder_t * Encoder);

/*码盘值数值清零处理*/
void EncoderValZero(Encoder_t * Encoder);

#endif
