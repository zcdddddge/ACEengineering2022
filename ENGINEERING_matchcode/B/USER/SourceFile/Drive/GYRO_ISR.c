#include "GYRO_ISR.h"

/*陀螺仪数据结构体*/
static GYRO_t	GYRO_G;

/*陀螺仪原始数据接收*/
static u8 gyro_receive[25];

/*************************************************************************************************
*名称:	Get_GYRO_Point
*功能:	获取陀螺仪数据结构体地址
*形参: 	无
*返回:	陀螺仪数据结构体地址 
*说明:	无
*************************************************************************************************/
GYRO_t *Get_GYRO_Point(void)
{
	return &GYRO_G;
}  	




/*************************************************************************************************
*名称:	gyro_deal
*功能:	陀螺仪数据处理
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
static void gyro_deal(void)
{
	if(gyro_receive[0]==0xfe&&gyro_receive[13]==0xee)
	{
		GYRO_G.gx  		= ((short)(gyro_receive[1]<<8|gyro_receive[2]))/100.0f;
		GYRO_G.gy  		= ((short)(gyro_receive[3]<<8|gyro_receive[4]))/100.0f;
		GYRO_G.gz  		= ((short)(gyro_receive[5]<<8|gyro_receive[6]))/100.0f;
		GYRO_G.pitch 	= ((short)(gyro_receive[7]<<8|gyro_receive[8]))/100.0f;
		GYRO_G.roll 	= ((short)(gyro_receive[9]<<8|gyro_receive[10]))/100.0f;
		GYRO_G.yaw 		= ((short)(gyro_receive[11]<<8|gyro_receive[12]))/100.0f;
	}
}



/*************************************************************************************************
*名称:	USART2_IRQHandler
*功能:	USART2中断
*形参: 	无
*返回:	无
*说明:	用于处理陀螺仪数据
*************************************************************************************************/
void USART2_IRQHandler(void)	
{
	u16 i;
	u8 num=0;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		DMA_Cmd(DMA1_Stream5, DISABLE);
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
		num=USART2->SR;
		num=USART2->DR;
		num=25-DMA_GetCurrDataCounter(DMA1_Stream5);
		for(i=0;i<num;i++)
		{
			gyro_receive[i]=gyro_rx_buffer[i];
		}
		DMA_SetCurrDataCounter(DMA1_Stream5,25);
		DMA_Cmd(DMA1_Stream5, ENABLE);
		gyro_deal();
	}
}





/*************************************************************************************************
*名称:	yaw_return_zero
*功能:	yaw角度归零
*形参: 	无
*返回:	无
*说明:	发送成功返回0，失败返回1
*************************************************************************************************/
u8 yaw_return_zero(void)
{
	u8 i;
	u8 SendBuff[2] = {0xfe,0xef};
  u16 clock = 0;
	
	for(i=0;i<2;i++)
	{
		clock = 0;
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET)
		{
			clock ++;
			if(clock > 0xf1f1)
				return 1;
		}
		USART_SendData(USART2,SendBuff[i]); 	  
	}
	return 0;
}



/*************************************************************************************************
*名称:	imu_calibration
*功能:	陀螺仪重新校准
*形参: 	无
*返回:	无
*说明:	发送成功返回0，失败返回1
*************************************************************************************************/
u8 imu_calibration(void)
{
	u8 i;
	u8 SendBuff[2] = {0x12,0x34};
	u16 clock = 0;
	
	for(i=0;i<2;i++)
	{
		clock = 0;
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET)
		{
			clock ++;
			if(clock > 0xf1f1)
				return 1;
		}
		USART_SendData(USART2,SendBuff[i]); 	  
	}
	
	return 0;
}

