#include "VISION_ISR.h"
#include "USART.h"

/*视觉数据结构体*/
static VISION_t Vision;
/*视觉数据*/
static u8 VISION_DATA[32];


/*************************************************************************************************
*名称:	Get_VISION_Point
*功能:	返回视觉结构体的指针
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
VISION_t *Get_VISION_Point(void)
{
	return &Vision;
}




/*************************************************************************************************
*名称:	VISION_MODE_SELECT
*功能:	转换数据为浮点型
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
static float TURN_FLOAT(uint8_t HighByte, uint8_t LowByte)
{
	float high = (float) (HighByte & 0x7f);
  float low  = (float) LowByte;
  if (HighByte & 0x80)									
  {
    return (high*256.0f + low) - 32768;
  }
  else
  {
    return (high*256.0f + low);
  }
}




/*************************************************************************************************
*名称:	VISION_MODE_SELECT
*功能:	处理视觉数据
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
static void VISION_DATA_DEAL(void) 
{
		u16 i;
		if(Vision.FLAG == 1)
		{
			for(i = 0;i < Vision.Vision_Len;i ++)
			{
				if(VISION_DATA[i]==0xFF && VISION_DATA[i+7]==0xFE)
				{
					Vision.AUTO_YAW   = (TURN_FLOAT(VISION_DATA[i+2],VISION_DATA[i+1])/100);		//自动打击的y轴角度计算/100
					Vision.AUTO_PITCH = (TURN_FLOAT(VISION_DATA[i+4],VISION_DATA[i+3])/100);		//自动打击的p轴角度计算
				
					Vision.DISTANCE   = (TURN_FLOAT(VISION_DATA[i+6],VISION_DATA[i+5])/100);		//距离
				
					Vision.YAW_ERR   = (Vision.AUTO_YAW - Vision.LAST_YAW);									  //与上一次相差的y轴角度
					Vision.PITCH_ERR = (Vision.AUTO_PITCH - Vision.LAST_PITCH);							  //与上一次相差的p轴角度
					Vision.LAST_PITCH= Vision.AUTO_PITCH;																			//记录前一次自动p轴角度
					Vision.LAST_YAW  = Vision.AUTO_YAW;  																			//记录前一次自动y轴角度
					i = i + 7;																																//一组八个字节数据
				}
			}
		}
		Vision.FLAG = 0;
}




/*************************************************************************************************
*名称:	USART3_IRQHandler
*功能:	USART3中断
*形参: 	无
*返回:	无
*说明:	用于处理视觉数据
*************************************************************************************************/
void USART3_IRQHandler(void)			
{
	u16 i;
	u8 num=0;
	
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		DMA_Cmd(DMA1_Stream1, DISABLE);
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
		
		USART3->SR;
		USART3->DR;
		num=256-DMA_GetCurrDataCounter(DMA1_Stream1);
		Vision.Vision_Len = num;
		
		for(i=0;i<num;i++)
		{
			VISION_DATA[i]=vision_rx_buffer[i];
		}
		VISION_DATA_DEAL();
		
		DMA_SetCurrDataCounter(DMA1_Stream1,32);
		DMA_Cmd(DMA1_Stream1, ENABLE);
		
		Vision.FLAG = 1;
	}
}



/*************************************************************************************************
*名称:	VISION_MODE_SELECT
*功能:	选择视觉的模式
*形参: 	u8 mode, u8 data
*返回:	无
*说明:	无
*************************************************************************************************/
u8 VISION_MODE_SELECT(u8 mode, u8 data)
{
	u8 i = 0;
	u8 SendBuff[4] = {0};
	uint16_t clock = 0;
	
	SendBuff[0] = 0xFF;
	SendBuff[1] = mode;
	SendBuff[2] = data;
	SendBuff[3] = 0xFE;
	
	for(i = 0;i < 4; i++)
	{
		clock = 0;
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET)
		{
				clock ++;
				if(clock > 0xf1f1)
					return 1;
		}			
		USART_SendData(USART3,(uint8_t)SendBuff[i]); 	 						
	}
	return 0;
}




/*************************************************************************************************
*名称:	VISION_INIT
*功能:	视觉结构体初始化
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
void VISION_INIT(VISION_t*vision)
{
	vision->AUTO_PITCH = vision->AUTO_YAW = 0.0f;
	vision->DISTANCE = vision->PITCH_ERR = vision->YAW_ERR = 0.0f;
	vision->LAST_PITCH = vision->LAST_YAW = 0.0f;
	vision->FLAG = vision->Vision_Len = 0;
}

