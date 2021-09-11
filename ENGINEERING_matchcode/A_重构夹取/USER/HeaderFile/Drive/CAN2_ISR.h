#ifndef __CAN2_ISR_H_
#define __CAN2_ISR_H_
#include "stm32f4xx.h"
#include "encoder.h"

/*CAN2���ͺ���������Ϊ���ͱ�ʶ��Ϊstdid������CAN_DATA[8]*/
void CAN2_To_Board(u8*CAN_DATA,int16_t stdid);

void CAN2_205_To_208_SEND(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208);

/*����can2���յ��ģ��ⲿ���ӵ���ֵ������ȡ���ذ��״ֵ̬
	����˵������*/
u8 *Return_CAN2_Board_Data(void);

Encoder_t * Return_Can2_201_208_Encoder(u8 ch);

#endif
