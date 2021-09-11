#ifndef __IWDG_H_
#define __IWDG_H_
#include "stm32f4xx_iwdg.h"
void IWDG_Init(u8 prer,u16 rlr);
void IWDG_Feed(void);
#endif
