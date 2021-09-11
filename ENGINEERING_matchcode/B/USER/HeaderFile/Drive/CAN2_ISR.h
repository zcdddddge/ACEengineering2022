#ifndef __CAN2_ISR_H_
#define __CAN2_ISR_H_
#include "stm32f4xx.h"
#include "encoder.h"


/*CAN2���ͺ���������Ϊ����������IDΪ201-204�ĵ��*/
void CAN2_201_To_204_SEND(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204);



/*CAN2���ͺ���������Ϊ����������IDΪ205-208�ĵ��*/
void CAN2_205_To_208_SEND(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207, int16_t ESC_208);



/*CAN2���ͺ���������Ϊ���ͱ�ʶ��Ϊstdid������CAN_DATA[8]*/
void CAN2_To_Board(u8*CAN_DATA,int16_t stdid);




/*����can2���յ��ģ��ⲿ���ӵ���ֵ������ȡ���ذ��״ֵ̬
	����˵������*/
u8 *Return_CAN2_Board_Data(void);



/*����Can2_205���ݴ���ָ��*/
Encoder_t * Return_Can2_205_Encoder(void);



/*����Can2_206���ݴ���ָ��*/
Encoder_t * Return_Can2_206_Encoder(void);


/*����Can2_207���ݴ���ָ��*/
Encoder_t * Return_Can2_207_Encoder(void);

Encoder_t * Return_Can2_201_208_Encoder(u8 ch);

#endif
