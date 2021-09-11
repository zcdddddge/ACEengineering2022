#ifndef __TIM_H_
#define __TIM_H_
#include "stm32f4xx.h"

/*TIM2定时器初始化，未开启中断*/
void TIM2_Init(uint16_t Period, uint16_t Prescaler);


/*TIM3定时器初始化，中断优先级为8*/
void TIM3_Init(uint16_t Period, uint16_t Prescaler);
#endif
