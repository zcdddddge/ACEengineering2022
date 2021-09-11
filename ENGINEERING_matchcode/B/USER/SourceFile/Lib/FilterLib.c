#include "FilterLib.h"

#ifndef NULL
#define NULL 0
#endif

/*************************************************************************************************
*����:	һ���˲��ṹ���ʼ������
*����:	��ʼ��һ���˲��ṹ��
*�β�: 	Filt_t*Filt,fp32 K
*����:	��
*˵��:	��
*************************************************************************************************/
void First_Order_Init(First_Order_t*First_Order,fp32 K)
{
	First_Order->Filt_K = K;
	First_Order->Input = First_Order->LastOuput = First_Order->Output = 0.0f;
}



/*************************************************************************************************
*����:	First_Order
*����:	һ���˲�����
*�β�: 	Filt_t*Filt,fp32 input
*����:	��
*˵��:	��
*************************************************************************************************/
void First_Order(First_Order_t*First_Order,fp32 input)
{
	if(First_Order == NULL) return;
	First_Order->Input = input;
	First_Order->Output = First_Order->Filt_K * First_Order->Input + (1 - First_Order->Filt_K) * First_Order->LastOuput;
	First_Order->LastOuput = First_Order->Output;
}
