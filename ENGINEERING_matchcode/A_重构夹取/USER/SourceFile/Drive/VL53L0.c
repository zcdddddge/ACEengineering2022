#include "vL53L0.h"
#include "USART.h"
#include "string.h"

VL53L0_t VL53L0;

/*����VL53L0���ṹ��*/
VL53L0_t* Return_VL53L0_t(void)
{
    return &VL53L0;
}


/*�ж�*/
u8 init = 3;
void USART6_IRQHandler(void)
{
    unsigned short i = 0;
    unsigned short Len = 0;

    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)					  //�����жϱ�־λ
    {
        DMA_Cmd(DMA2_Stream1, DISABLE);															  //�ر�DMA,��ֹ�����ڼ�������

        while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);

        USART6->SR;
        USART6->DR;
        Len = 128 - DMA_GetCurrDataCounter(DMA2_Stream1);   					//DMA_GetCurrDataCounter���ڻ�ô�����ʣ��ĵ�Ԫ��

        for(i = 0; i < Len; i ++)
        {
            VL53L0.data[i] = vL53L0_rx_buffer[i];
        }

        DMA_SetCurrDataCounter(DMA2_Stream1, 128);           					//����Ҫ��������ݵ�Ԫ��
        DMA_Cmd(DMA2_Stream1, ENABLE);																//����DMA
        VL53L0.receive_flag = 1;
    }
}


/*����VL53L0�������*/
void VL53L0_Data_Deal(void)
{
    //u8 *p;
    u16 temp = 0;

    if(VL53L0.receive_flag == 1)
    {
//			p = (u8*)strstr((char*)VL53L0.data,"d:");
//			while(*p != 'm')
//			{
//				if(*p>='0' && *p<='9')
//				{
//					VL53L0.distance = VL53L0.distance*10 + (*p - '0');
//				}
//
//				p++;
//			}
        if(VL53L0.data[29] >= 48 && VL53L0.data[29] <= 57)
        {
            temp += (VL53L0.data[29] - 48) * 1000;
        }

        if(VL53L0.data[30] >= 48 && VL53L0.data[30] <= 57)
        {
            temp += (VL53L0.data[30] - 48) * 100;
        }

        if(VL53L0.data[31] >= 48 && VL53L0.data[31] <= 57)
        {
            temp += (VL53L0.data[31] - 48) * 10;
        }

        if(VL53L0.data[32] >= 48 && VL53L0.data[32] <= 57)
        {
            temp += (VL53L0.data[32] - 48);
        }

        VL53L0.distance = (((float)(temp) / 10) - 4.1);

        if(init == 3)
        {
            init ++;
            VL53L0.InitDistance = VL53L0.distance;
        }

        VL53L0.receive_flag = 0;
    }
}
