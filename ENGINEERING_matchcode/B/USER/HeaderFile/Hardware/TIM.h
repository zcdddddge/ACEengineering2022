#ifndef __TIM_H_
#define __TIM_H_
#include "stm32f4xx.h"

/*TIM2��ʱ����ʼ����δ�����ж�*/
void TIM2_Init(uint16_t Period, uint16_t Prescaler);


/*TIM3��ʱ����ʼ�����ж����ȼ�Ϊ8*/
void TIM3_Init(uint16_t Period, uint16_t Prescaler);
#endif
