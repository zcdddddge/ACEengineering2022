#ifndef	__FILTERLIB_H_
#define __FILTERLIB_H_
#include "stm32f4xx.h"

#define fp32 float


/*一阶滤波函数*/
typedef __packed struct
{
	fp32 Input;
	fp32 LastOuput;
	fp32 Output;
	fp32 Filt_K;
}First_Order_t;



/*一阶滤波初始化*/
void First_Order_Init(First_Order_t*First_Order,fp32 K);



/*一阶滤波函数*/
void First_Order(First_Order_t*First_Order,fp32 input);

#endif
