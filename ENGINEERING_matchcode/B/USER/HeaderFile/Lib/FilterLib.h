#ifndef	__FILTERLIB_H_
#define __FILTERLIB_H_
#include "stm32f4xx.h"

#define fp32 float


/*һ���˲�����*/
typedef __packed struct
{
	fp32 Input;
	fp32 LastOuput;
	fp32 Output;
	fp32 Filt_K;
}First_Order_t;



/*һ���˲���ʼ��*/
void First_Order_Init(First_Order_t*First_Order,fp32 K);



/*һ���˲�����*/
void First_Order(First_Order_t*First_Order,fp32 input);

#endif
