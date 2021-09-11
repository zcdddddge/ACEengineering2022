#include "PYR_ISR.h"
#include "USART.h"

PYR_t PYR;

/*返回底盘陀螺仪数据指针*/
PYR_t* Return_PYR_t(void)
{
	return &PYR;
}


/*底盘陀螺仪数据处理函数*/
static void PYR_Deal(u8 num)
{
	u8 i;
	u16 n;
	u8 sum;
	for(i=0; i<num; i++)
	{
		if((PYR.data[i] == 0x55) && (PYR.data[i+11] == 0x55) && (PYR.data[i+22] == 0x55))	//校验3组数据头
		{
			if(PYR.data[i+23] == 0x53)
			{
				n = i+22;		//重置坐标
				sum = 0x55 + 0x53 + PYR.data[n+2] + PYR.data[n+3] + PYR.data[n+4] + 
				PYR.data[n+5] + PYR.data[n+6] + PYR.data[n+7] + PYR.data[n+8] 
				+ PYR.data[n+9];
				if(PYR.data[n+10] != sum)	//检查校验和
				return;
					
				//PYR.Pitch = ((PYR.data[n+3]<< 8) | PYR.data[n+2])/32768.0*180;
				//PYR.Roll = ((PYR.data[n+5]<< 8) | PYR.data[n+4])/32768.0*180; 
				PYR.Yaw = ((PYR.data[n+7]<< 8) | PYR.data[n+6])/32768.0*180;  // 角速度计算|偏航角(z轴) 
				PYR.Yaw_Var = PYR.Yaw - PYR.Yaw_Lock;
				if(PYR.Yaw_Var > 300)
				{
					PYR.Yaw_Var = -(360 - PYR.Yaw + PYR.Yaw_Lock);
				}
				else if(PYR.Yaw_Var <  -300)
				{
					PYR.Yaw_Var = 360 + PYR.Yaw - PYR.Yaw_Lock;
				}
				PYR.flag = 1;
			}
		}
	}
	
}
	
/*uart4中断函数*/
void UART4_IRQHandler(void)
{
	u16 i;
	u8 num=0;
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
	{
		DMA_Cmd(DMA1_Stream2, DISABLE);
		while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
		num	=	UART4->SR;
		num	=	UART4->DR;
		num	=	64-DMA_GetCurrDataCounter(DMA1_Stream5);
		for(i=0;i<num;i++)
		{
			PYR.data[i] = PYR_rx_buffer[i];
		}
		PYR.flag = 2;
		PYR_Deal(num);
		DMA_SetCurrDataCounter(DMA1_Stream2,64);
		DMA_Cmd(DMA1_Stream2, ENABLE);
	}
}

