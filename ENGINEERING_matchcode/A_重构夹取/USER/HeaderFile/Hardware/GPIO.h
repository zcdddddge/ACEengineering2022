#ifndef __GPIO_H_
#define __GPIO_H_
#include "stm32f4xx.h"
				 
/*LED初始化，IO对应：PE7，PE14*/
void Led_Init(void);

/*传感器IO初始化，IO对应PA1，PA9*/
void SENSOR_Init(void);

/*激光IO初始化，IO：PA12*/
void LASER_Init(void);

/*开关IO初始化，IO对应：PC3，PC10，PC11*/
void SWITCH_Init(void);
#endif
