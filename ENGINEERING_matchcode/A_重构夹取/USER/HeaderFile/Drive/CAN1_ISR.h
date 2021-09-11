#ifndef __CAN1_ISR_H_
#define __CAN1_ISR_H_
#include "stm32f4xx.h"
#include "encoder.h"


/*CAN1发送函数，功能为发送数据至ID为201-204的电调*/
void CAN1_201_To_204_SEND(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204);



/*CAN1发送函数，功能为发送数据至ID为205-208的电调*/
void CAN1_205_To_208_SEND(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207, int16_t ESC_208);



/*CAN1发送函数，功能为发送标识符为stdid的数据CAN_DATA[8]*/
void CAN1_To_Board(u8*CAN_DATA,int16_t stdid);


/*返回Can1_201_208数据处理指针,返回数值为20(ch)*/
Encoder_t * Return_Can1_201_208_Encoder(u8 ch);
	
#endif
