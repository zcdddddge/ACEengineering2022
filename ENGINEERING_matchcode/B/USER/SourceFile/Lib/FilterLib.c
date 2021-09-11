#include "FilterLib.h"

#ifndef NULL
#define NULL 0
#endif

/*************************************************************************************************
*名称:	一阶滤波结构体初始化函数
*功能:	初始化一阶滤波结构体
*形参: 	Filt_t*Filt,fp32 K
*返回:	无
*说明:	无
*************************************************************************************************/
void First_Order_Init(First_Order_t*First_Order,fp32 K)
{
	First_Order->Filt_K = K;
	First_Order->Input = First_Order->LastOuput = First_Order->Output = 0.0f;
}



/*************************************************************************************************
*名称:	First_Order
*功能:	一阶滤波函数
*形参: 	Filt_t*Filt,fp32 input
*返回:	无
*说明:	无
*************************************************************************************************/
void First_Order(First_Order_t*First_Order,fp32 input)
{
	if(First_Order == NULL) return;
	First_Order->Input = input;
	First_Order->Output = First_Order->Filt_K * First_Order->Input + (1 - First_Order->Filt_K) * First_Order->LastOuput;
	First_Order->LastOuput = First_Order->Output;
}
