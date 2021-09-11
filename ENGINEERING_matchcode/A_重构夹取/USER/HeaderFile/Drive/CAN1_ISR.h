#ifndef __CAN1_ISR_H_
#define __CAN1_ISR_H_
#include "stm32f4xx.h"
#include "encoder.h"


/*CAN1���ͺ���������Ϊ����������IDΪ201-204�ĵ��*/
void CAN1_201_To_204_SEND(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204);



/*CAN1���ͺ���������Ϊ����������IDΪ205-208�ĵ��*/
void CAN1_205_To_208_SEND(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207, int16_t ESC_208);



/*CAN1���ͺ���������Ϊ���ͱ�ʶ��Ϊstdid������CAN_DATA[8]*/
void CAN1_To_Board(u8*CAN_DATA,int16_t stdid);


/*����Can1_201_208���ݴ���ָ��,������ֵΪ20(ch)*/
Encoder_t * Return_Can1_201_208_Encoder(u8 ch);
	
#endif
