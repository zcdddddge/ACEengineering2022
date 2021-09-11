#include "usart6.h"
#include "parameter.h"
#include "fifo_buff.h"

#define USART6_RX_LEN 512
#define USART6_TX_LEN 256
uint8_t Usart6_Rx[USART6_RX_LEN] = {0};
uint8_t Usart6_Tx[USART6_TX_LEN] = {0};
uint8_t Usart6_Tx_Buffer[USART6_TX_LEN] = {0};
volatile unsigned char vL53L0_rx_buffer[128];

fifo_rx_def fifo_usart_rx_6;
fifo_rx_def fifo_usart_tx_6;

uint8_t DMA2_Stream6_working = 0;

uint8_t (*usart6_callback)(uint8_t *, uint16_t);

void usart6_init(uint8_t usart_NVIC)
{
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_InitTypeDef USART6_InitStruct;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStruct);

		USART_DeInit(USART6);
		USART6_InitStruct.USART_BaudRate = 115200;
		USART6_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART6_InitStruct.USART_StopBits = USART_StopBits_1;
		USART6_InitStruct.USART_Parity = USART_Parity_No; //����żУ��
		USART6_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART6, &USART6_InitStruct);

		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = usart_NVIC;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&NVIC_InitStructure);
		
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); //��������ж�
		USART_ITConfig(USART6, USART_IT_TC, DISABLE);
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
		USART_Cmd(USART6, ENABLE);
	}
	/* DMA���� */
	{
		DMA_InitTypeDef DMA_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		
        //��������
        DMA_DeInit(DMA2_Stream6); //����Ϊȱʡֵ
		DMA_StructInit(&DMA_InitStruct);
        DMA_InitStruct.DMA_Channel = DMA_Channel_5;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);   //Դ��ַ
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(vL53L0_rx_buffer);          //Ŀ�ĵ�ַ
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStruct.DMA_BufferSize = USART6_TX_LEN;                       //�������ݵĻ����С
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
        DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;                           //��������ģʽ
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream6, &DMA_InitStruct);
		
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
		DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);  //������������ж�
        DMA_Cmd(DMA2_Stream6, DISABLE);
        USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE); //ʹ�ܴ���2��DMA����
		
		if (fifo_init(&fifo_usart_tx_6, Usart6_Tx_Buffer, USART6_TX_LEN) == -1)
		{
			// ���� 2 ���ݴη�
		}
	}
	/* DMA���� */
    {
		DMA_InitTypeDef DMA_InitStruct;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		
        //��������
        DMA_DeInit(DMA2_Stream1); //����Ϊȱʡֵ
		DMA_StructInit(&DMA_InitStruct);
        DMA_InitStruct.DMA_Channel = DMA_Channel_5;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);   //Դ��ַ
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart6_Rx);          //Ŀ�ĵ�ַ
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStruct.DMA_BufferSize = USART6_RX_LEN;                       //�������ݵĻ����С
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //������ѭ������ģʽ
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        //DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        //DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        //DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;
        //DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream1, &DMA_InitStruct);
		
        DMA_Cmd(DMA2_Stream1, ENABLE);
        USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    }
}


//void USART6_IRQHandler(void)
//{
//    __IO uint16_t Len = 0;

//	//��������ж�
//	/*
//	 * DMA�ж�ʱ��ֻ��ʾ��Ҫ���͵����������ֽ�ȫ�����͵����ڵķ������ݼĴ������ˡ�
//	 * ��ʱ����ʵ���ϻ���2���ֽڲ�δ������ɣ����ݼĴ�������λ�Ĵ����е�2���ֽڻ���Ҫ���ͣ������ܹرմ��ڷ��͡�
//	 * ͬ�������485�л����򣬱���Ҫ�ȵ�������ɣ�Ҳ������λ�Ĵ����������-TC��־��λ��
//	*/
//    if (USART_GetITStatus(USART6, USART_IT_TC) == SET)
//    {
//		USART_ClearITPendingBit(USART6, USART_IT_TC);
//        USART_ITConfig(USART6, USART_IT_TC, DISABLE);
//		DMA2_Stream6_working = 0;
//    }
//    if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) //�����жϱ�־λ
//    {
//        DMA_Cmd(DMA2_Stream1, DISABLE); //�ر�DMA,��ֹ�����ڼ�������

//        while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE)
//            ;

//        Len = USART6->SR;
//        Len = USART6->DR;
//        Len = DMA_GetCurrDataCounter(DMA2_Stream1); //DMA_GetCurrDataCounter���ڻ�ô�����ʣ��ĵ�Ԫ��

//        if (usart6_callback != NULL)
//            usart6_callback(Usart6_Rx, Len);

//        memset(Usart6_Rx, 0, USART6_RX_LEN);

//        DMA_SetCurrDataCounter(DMA2_Stream1, USART6_RX_LEN); //����Ҫ��������ݵ�Ԫ��
//        DMA_Cmd(DMA2_Stream1, ENABLE);                       //����DMA
//    }
//}


/**
  * @brief          ��������ж�
  * @param[in]      void
  * @retval         void
  */
void DMA2_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream6, DMA_IT_TCIF6);
        DMA_Cmd(DMA2_Stream6, DISABLE); //��ʹ�ܴ���
		USART_ITConfig(USART6, USART_IT_TC, ENABLE);
    }
}

/**
  * @brief          ������+DMA ����
  * @param[in]      void
  * @retval         void
  */
uint32_t usart6_dma_send(uint8_t *data, uint16_t len)
{	
    uint32_t result = fifo_write_buff(&fifo_usart_tx_6, data, len); //�����ݷ���ѭ��������
	
    //result != 0 ˵���������ݳɹ� DMA2_Stream6_working == 0 ˵������������û������
    if (DMA2_Stream6_working == 0)
    {
        len = fifo_read_buff(&fifo_usart_tx_6, Usart6_Tx, USART6_TX_LEN); //��ѭ����������ȡ����
		
        DMA_SetCurrDataCounter(DMA2_Stream6, len); //�趨���䳤��
        DMA_Cmd(DMA2_Stream6, ENABLE); //ʹ�ܴ���
		
        DMA2_Stream6_working = 1;
    }
	
	if (result == len)
	{
		return len;
	}
	else
	{
		return result;
	}
}


