#ifndef __CAN2_ISR_H_
#define __CAN2_ISR_H_
#include "stm32f4xx.h"
#include "encoder.h"


/*CAN2发送函数，功能为发送数据至ID为201-204的电调*/
void CAN2_201_To_204_SEND(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204);



/*CAN2发送函数，功能为发送数据至ID为205-208的电调*/
void CAN2_205_To_208_SEND(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207, int16_t ESC_208);



/*CAN2发送函数，功能为发送标识符为stdid的数据CAN_DATA[8]*/
void CAN2_To_Board(u8*CAN_DATA,int16_t stdid);




/*返回can2接收到的，外部板子的数值，即夹取主控板的状态值
	具体说明暂无*/
u8 *Return_CAN2_Board_Data(void);



/*返回Can2_205数据处理指针*/
Encoder_t * Return_Can2_205_Encoder(void);



/*返回Can2_206数据处理指针*/
Encoder_t * Return_Can2_206_Encoder(void);


/*返回Can2_207数据处理指针*/
Encoder_t * Return_Can2_207_Encoder(void);

Encoder_t * Return_Can2_201_208_Encoder(u8 ch);

#endif
