#ifndef __PYR_ISR_H_
#define __PYR_ISR_H_
#include "stm32f4xx.h"

typedef __packed struct
{
	u8 data[128];
	float Roll;
	float Pitch;
	float	Yaw;
	float Yaw_Lock;
	float Yaw_Var;
	u8 flag;
}PYR_t;

/*返回底盘PYR数据指针*/
PYR_t* Return_PYR_t(void);

#endif
