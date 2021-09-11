#include "VISION_ISR.h"
#include "USART.h"

/*�Ӿ����ݽṹ��*/
static VISION_t Vision;
/*�Ӿ�����*/
static u8 VISION_DATA[32];


/*************************************************************************************************
*����:	Get_VISION_Point
*����:	�����Ӿ��ṹ���ָ��
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
VISION_t *Get_VISION_Point(void)
{
	return &Vision;
}




/*************************************************************************************************
*����:	VISION_MODE_SELECT
*����:	ת������Ϊ������
*�β�: 	��
*����:	��
*˵��:	��
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
*����:	VISION_MODE_SELECT
*����:	�����Ӿ�����
*�β�: 	��
*����:	��
*˵��:	��
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
					Vision.AUTO_YAW   = (TURN_FLOAT(VISION_DATA[i+2],VISION_DATA[i+1])/100);		//�Զ������y��Ƕȼ���/100
					Vision.AUTO_PITCH = (TURN_FLOAT(VISION_DATA[i+4],VISION_DATA[i+3])/100);		//�Զ������p��Ƕȼ���
				
					Vision.DISTANCE   = (TURN_FLOAT(VISION_DATA[i+6],VISION_DATA[i+5])/100);		//����
				
					Vision.YAW_ERR   = (Vision.AUTO_YAW - Vision.LAST_YAW);									  //����һ������y��Ƕ�
					Vision.PITCH_ERR = (Vision.AUTO_PITCH - Vision.LAST_PITCH);							  //����һ������p��Ƕ�
					Vision.LAST_PITCH= Vision.AUTO_PITCH;																			//��¼ǰһ���Զ�p��Ƕ�
					Vision.LAST_YAW  = Vision.AUTO_YAW;  																			//��¼ǰһ���Զ�y��Ƕ�
					i = i + 7;																																//һ��˸��ֽ�����
				}
			}
		}
		Vision.FLAG = 0;
}




/*************************************************************************************************
*����:	USART3_IRQHandler
*����:	USART3�ж�
*�β�: 	��
*����:	��
*˵��:	���ڴ����Ӿ�����
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
*����:	VISION_MODE_SELECT
*����:	ѡ���Ӿ���ģʽ
*�β�: 	u8 mode, u8 data
*����:	��
*˵��:	��
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
*����:	VISION_INIT
*����:	�Ӿ��ṹ���ʼ��
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
void VISION_INIT(VISION_t*vision)
{
	vision->AUTO_PITCH = vision->AUTO_YAW = 0.0f;
	vision->DISTANCE = vision->PITCH_ERR = vision->YAW_ERR = 0.0f;
	vision->LAST_PITCH = vision->LAST_YAW = 0.0f;
	vision->FLAG = vision->Vision_Len = 0;
}

