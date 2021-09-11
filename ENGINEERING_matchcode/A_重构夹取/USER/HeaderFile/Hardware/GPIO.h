#ifndef __GPIO_H_
#define __GPIO_H_
#include "stm32f4xx.h"
				 
/*LED��ʼ����IO��Ӧ��PE7��PE14*/
void Led_Init(void);

/*������IO��ʼ����IO��ӦPA1��PA9*/
void SENSOR_Init(void);

/*����IO��ʼ����IO��PA12*/
void LASER_Init(void);

/*����IO��ʼ����IO��Ӧ��PC3��PC10��PC11*/
void SWITCH_Init(void);
#endif
