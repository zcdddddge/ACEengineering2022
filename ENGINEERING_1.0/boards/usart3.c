#include "usart3.h"
#include "parameter.h"
#include "fifo_buff.h"


#define USART3_RX_LEN 32
#define USART3_TX_LEN 32
uint8_t Usart3_Rx[USART3_RX_LEN] = {0};
uint8_t Usart3_Tx[USART3_TX_LEN] = {0};
uint8_t Usart3_Tx_Buffer[USART3_TX_LEN] = {0};

fifo_rx_def fifo_usart_rx_3;
fifo_rx_def *pfifo_3 = &fifo_usart_rx_3;
fifo_rx_def fifo_usart_tx_3;

uint8_t DMA1_Stream3_working = 0;

void usart3_init(uint8_t usart_NVIC)
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    //ʹ��USART3ʱ��
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); //GPIOD8����ΪUSART3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); //GPIOD9����ΪUSART3
	
    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        NVIC_InitTypeDef NVIC_InitStructure;
        USART_InitTypeDef USART3_InitStruct;

        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOD, &GPIO_InitStruct);

        USART_DeInit(USART3);
        USART3_InitStruct.USART_BaudRate = 115200;
        USART3_InitStruct.USART_WordLength = USART_WordLength_8b;
        USART3_InitStruct.USART_StopBits = USART_StopBits_1;
        USART3_InitStruct.USART_Parity = USART_Parity_No; //����żУ��
        USART3_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART3_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART3, &USART3_InitStruct);

        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //DMA1_Stream1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = usart_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_Init(&NVIC_InitStructure);
		
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //��������ж�
		USART_ITConfig(USART3, USART_IT_TC, DISABLE);
		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART3, ENABLE);
    }
    /* -------------- Configure DMA -----------------------------------------*/
    {
        //��������
        DMA_InitTypeDef DMA_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		
		DMA_Cmd(DMA1_Stream3, DISABLE); //�ر�DMA
        DMA_DeInit(DMA1_Stream3); //����Ϊȱʡֵ
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);   //Դ��ַ
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart3_Tx);          //Ŀ�ĵ�ַ
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStruct.DMA_BufferSize = USART3_TX_LEN;                       //�������ݵĻ����С
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
        DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;                         //������ѭ������ģʽ
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single; //DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream3, &DMA_InitStruct);
		
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
		DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);  //������������ж�
        DMA_Cmd(DMA1_Stream3, DISABLE);
        USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE); //ʹ�ܴ���3��DMA����

		if (fifo_init(&fifo_usart_tx_3, Usart3_Tx_Buffer, USART3_TX_LEN) == -1)
		{
			// ���� 2 ���ݴη�
		}
    }
	{
		DMA_InitTypeDef DMA_InitStruct;
		
        // ��������
        DMA_DeInit(DMA1_Stream1); //����Ϊȱʡֵ
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);   //Դ��ַ
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart3_Rx);          //Ŀ�ĵ�ַ
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStruct.DMA_BufferSize = USART3_RX_LEN;                       //�������ݵĻ����С
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //������ѭ������ģʽ
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal; //DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream1, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream1, ENABLE);
        USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);		
		
		if (fifo_init(pfifo_3, Usart3_Rx, USART3_RX_LEN) == -1)
		{
			// ���� 2 ���ݴη�
		}
	}
}

void USART3_IRQHandler(void)
{
    __IO uint8_t Len = 0;

	//��������ж�
	/*
	 * DMA�ж�ʱ��ֻ��ʾ��Ҫ���͵����������ֽ�ȫ�����͵����ڵķ������ݼĴ������ˡ�
	 * ��ʱ����ʵ���ϻ���2���ֽڲ�δ������ɣ����ݼĴ�������λ�Ĵ����е�2���ֽڻ���Ҫ���ͣ������ܹرմ��ڷ��͡�
	 * ͬ�������485�л����򣬱���Ҫ�ȵ�������ɣ�Ҳ������λ�Ĵ����������-TC��־��λ��
	*/
    if (USART_GetITStatus(USART3, USART_IT_TC) == SET)
    {
		USART_ClearITPendingBit(USART3, USART_IT_TC);
        USART_ITConfig(USART3, USART_IT_TC, DISABLE);
		DMA1_Stream3_working = 0;
    }
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) //�����жϱ�־λ
    {
        Len = USART3->SR;
        Len = USART3->DR;
        Len = DMA_GetCurrDataCounter(DMA1_Stream1);

		if (pfifo_3 != 0)
		{
			// LenΪ��ǰ��������
			pfifo_3->in += ((pfifo_3->last_cnt - Len) & (pfifo_3->size - 1)); //����in
			pfifo_3->last_cnt = Len;

			if ((pfifo_3->in - pfifo_3->out) > pfifo_3->size)
			{
				pfifo_3->out = pfifo_3->in; // ��ջ��棬ע�⸳ֵ˳��pfifo->in = pfifo->out �Ǵ����
				pfifo_3->error |= FIFO_DMA_ERROR_RX_FULL;
			}
		}
		else
		{
			pfifo_3->error |= FIFO_DMA_ERROR_RX_POINT_NULL;
		}
    }
}

/**
  * @brief          ��������ж�
  * @param[in]      void
  * @retval         void
  */
void DMA1_Stream3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)
    {
        DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);
        DMA_Cmd(DMA1_Stream3, DISABLE); //��ʹ�ܴ���
		USART_ITConfig(USART3, USART_IT_TC, ENABLE);
    }
}

/**
  * @brief          ������+DMA ����
  * @param[in]      void
  * @retval         void
  */
uint32_t usart3_dma_send(uint8_t *data, uint16_t len)
{	
    uint32_t result = fifo_write_buff(&fifo_usart_tx_3, data, len); //�����ݷ���ѭ��������
	
    //result != 0 ˵���������ݳɹ� DMA2_Stream6_working == 0 ˵������������û������
    if (DMA1_Stream3_working == 0)
    {
        len = fifo_read_buff(&fifo_usart_tx_3, Usart3_Tx, USART3_TX_LEN); //��ѭ����������ȡ����
		
        DMA_SetCurrDataCounter(DMA1_Stream3, len); //�趨���䳤��
        DMA_Cmd(DMA1_Stream3, ENABLE); //ʹ�ܴ���
		
        DMA1_Stream3_working = 1;
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


