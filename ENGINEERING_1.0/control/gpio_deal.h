#ifndef __GPIO_DEAL_H_
#define __GPIO_DEAL_H_
#include "stm32f4xx.h"

typedef __packed struct
{
	u8 Smooth_L;
	u8 Smooth_R;
	u8 Stretch_F;
}Sensor_t;

/*获取传感器数值*/
void Get_Sensor_Val(void);

u8 Return_Smooth_L_Val(void);
u8 Return_Smooth_R_Val(void);

/*返回传感器数值结构体*/
Sensor_t* Return_Sensor_t(void);

#endif
