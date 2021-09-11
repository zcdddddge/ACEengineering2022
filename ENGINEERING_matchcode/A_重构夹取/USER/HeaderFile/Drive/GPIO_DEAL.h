#ifndef __GPIO_DEAL_H_
#define __GPIO_DEAL_H_
#include "stm32f4xx.h"

typedef __packed struct
{
	u8 Smooth_L;
	u8 Smooth_R;
	u8 Stretch_F;
}Sensor_t;

/*��ȡ��������ֵ*/
void Get_Sensor_Val(void);

/*���ش�������ֵ�ṹ��*/
Sensor_t* Return_Sensor_t(void);

u8 get_sensor_flag(void);

#endif
