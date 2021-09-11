#include "usart1.h"
#include "parameter.h"
#include "fifo_buff.h"

#define RC_RX_LEN 18
uint8_t RC_Rx[RC_RX_LEN];

#define USART1_IMU_RX_LEN 32
#define USART1_IMU_TX_LEN 32
uint8_t Usart1_IMU_Rx[USART1_IMU_RX_LEN] = {0};
uint8_t Usart1_IMU_Tx[USART1_IMU_TX_LEN] = {0};
uint8_t Usart1_IMU_Tx_Buffer[USART1_IMU_TX_LEN] = {0};

fifo_rx_def fifo_usart_rx_1;
fifo_rx_def *pfifo_1 = &fifo_usart_rx_1;
fifo_rx_def fifo_usart_tx_1;

uint8_t DMA2_Stream7_working = 0;

void (*rc_callback)(volatile const uint8_t *);

void usart1_init(uint8_t usart_NVIC, uint16_t usart1_mode)
{
	/* ң���� */
    if (usart1_mode == 1)
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure; //�ṹ������

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //ʱ��ʹ��

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStructure); //����io������

        USART_DeInit(USART1);
        USART_InitStructure.USART_BaudRate = 100000; //ң�ؽ��ղ�����
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_Mode = USART_Mode_Rx; //����
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART1, &USART_InitStructure);

		USART_ITConfig(USART1, USART_IT_IDLE, DISABLE); //����ң������ʱ��Ҫ���ڿ����ж�
        USART_Cmd(USART1, ENABLE);                     //ʹ��USART1
        USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); //ʹ�ܴ���DMA����

        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = usart_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure); //�ж����ȼ�

        DMA_Cmd(DMA2_Stream5, DISABLE); //�ر�DMA
        while (DMA2_Stream5->CR & DMA_SxCR_EN); //�ȴ� DMA ������

        DMA_DeInit(DMA2_Stream5); //����Ϊȱʡֵ
        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   //Դ��ַ
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RC_Rx;                //Ŀ�ĵ�ַ
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStructure.DMA_BufferSize = RC_RX_LEN;                           //�������ݵĻ����С
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //���ݿ��Ϊ8λ
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         //������ѭ������ģʽ
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream5, &DMA_InitStructure);

        DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE); //dma������ɲ����ж�
        DMA_Cmd(DMA2_Stream5, ENABLE); //����DMA
    }
	/* ������ */
    else if (usart1_mode == 2)
    {
        /* -------------- Enable Module Clock Source ----------------------------*/
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //ʹ��GPIOAʱ��
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                      //ʹ��USART1ʱ��

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //GPIOA9����ΪUSART1
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10����ΪUSART1

        /* -------------- Configure GPIO ---------------------------------------*/
        {
            GPIO_InitTypeDef GPIO_InitStruct;
            NVIC_InitTypeDef NVIC_InitStructure;
            USART_InitTypeDef USART1_InitStruct;

            //USART1�˿�����
            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;            //���ù���
            GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;       //�ٶ�50MHz
            GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;          //���츴�����
            GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;            //����
            GPIO_Init(GPIOA, &GPIO_InitStruct);                  //��ʼ��PA9��PA10

            //USART1 ��ʼ������
            USART_DeInit(USART1);
            USART1_InitStruct.USART_BaudRate = 115200;                                    //����������
            USART1_InitStruct.USART_WordLength = USART_WordLength_8b;                     //�ֳ�Ϊ8λ���ݸ�ʽ
            USART1_InitStruct.USART_StopBits = USART_StopBits_1;                          //һ��ֹͣλ
            USART1_InitStruct.USART_Parity = USART_Parity_No;                             //����żУ��λ
            USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
            USART1_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //�շ�ģʽ
            USART_Init(USART1, &USART1_InitStruct);                                       //��ʼ������1

            USART_Cmd(USART1, ENABLE); //ʹ�ܴ���1
            USART_ClearFlag(USART1, USART_FLAG_TC);

            //Usart1 NVIC ����
            NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                  //����1�ж�ͨ��
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
            NVIC_Init(&NVIC_InitStructure);                 //����ָ���Ĳ�����ʼ��VIC�Ĵ���
			
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //��������ж�
			USART_ITConfig(USART1, USART_IT_TC, DISABLE);
            USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
        }

        /* -------------- Configure DMA -----------------------------------------*/
		/* ���� */
        {
            DMA_InitTypeDef DMA_InitStruct;
			NVIC_InitTypeDef NVIC_InitStructure;
			
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

            //��������
			DMA_Cmd(DMA2_Stream7, DISABLE); //�ر�DMA
            DMA_DeInit(DMA2_Stream7); //����Ϊȱʡֵ
            DMA_InitStruct.DMA_Channel = DMA_Channel_4;
            DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   //Դ��ַ
            DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart1_IMU_Tx);      //Ŀ�ĵ�ַ
            DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //���ݴ��䷽��Ϊ���赽�ڴ�
            DMA_InitStruct.DMA_BufferSize = USART1_IMU_TX_LEN;                   //�������ݵĻ����С
            DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
            DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
            DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
            DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
            DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;                           //����������ģʽ
            DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
            DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
            DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
            DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
            DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
            DMA_Init(DMA2_Stream7, &DMA_InitStruct);
			
			NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);  //������������ж�
            DMA_Cmd(DMA2_Stream7, DISABLE);
            USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); //ʹ�ܴ��ڵ�DMA����
			
			if (fifo_init(&fifo_usart_tx_1, Usart1_IMU_Tx_Buffer, USART1_IMU_TX_LEN) == -1)
			{
				// ���� 2 ���ݴη�
			}
        }
		// ��������
		{
			DMA_InitTypeDef DMA_InitStruct;
			
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			
			DMA_Cmd(DMA2_Stream5, DISABLE); //�ر�DMA
			while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE)
				;
            DMA_DeInit(DMA2_Stream5); //����Ϊȱʡֵ
            DMA_InitStruct.DMA_Channel = DMA_Channel_4;
            DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   //Դ��ַ
            DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart1_IMU_Rx);      //Ŀ�ĵ�ַ
            DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���ݴ��䷽��Ϊ���赽�ڴ�
            DMA_InitStruct.DMA_BufferSize = USART1_IMU_RX_LEN;                   //�������ݵĻ����С
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
            DMA_Init(DMA2_Stream5, &DMA_InitStruct);
			
            DMA_Cmd(DMA2_Stream5, ENABLE);
            USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	

			if (fifo_init(pfifo_1, Usart1_IMU_Rx, USART1_IMU_RX_LEN) == -1)
			{
				// ���� 2 ���ݴη�
			}
		}
    }
}

void USART1_IRQHandler(void) // ���������ж�
{
    __IO uint8_t Len = 0;
	
	//��������ж�
	/*
	 * DMA�ж�ʱ��ֻ��ʾ��Ҫ���͵����������ֽ�ȫ�����͵����ڵķ������ݼĴ������ˡ�
	 * ��ʱ����ʵ���ϻ���2���ֽڲ�δ������ɣ����ݼĴ�������λ�Ĵ����е�2���ֽڻ���Ҫ���ͣ������ܹرմ��ڷ��͡�
	 * ͬ�������485�л����򣬱���Ҫ�ȵ�������ɣ�Ҳ������λ�Ĵ����������-TC��־��λ��
	*/
    if (USART_GetITStatus(USART1, USART_IT_TC) == SET)
    {
		USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
		DMA2_Stream7_working = 0;
    }
	//���߿����ж�
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //�����жϱ�־λ
    {
        //DMA_Cmd(DMA2_Stream5, DISABLE); //�ر�DMA,��ֹ�����ڼ�������
		//DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
        //while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);

        Len = USART1->SR; //���RXNE��־
        Len = USART1->DR; //��USART_IT_IDLE��־  
        Len = DMA_GetCurrDataCounter(DMA2_Stream5); //��ȡ��ǰʣ����������С�ĺ���

		if (pfifo_1 != 0)
		{
			// LenΪ��ǰ��������
			pfifo_1->in += ((pfifo_1->last_cnt - Len) & (pfifo_1->size - 1)); //����in
			pfifo_1->last_cnt = Len;

			if ((pfifo_1->in - pfifo_1->out) > pfifo_1->size)
			{
				pfifo_1->out = pfifo_1->in; // ��ջ��棬ע�⸳ֵ˳��pfifo->in = pfifo->out �Ǵ����
				pfifo_1->error |= FIFO_DMA_ERROR_RX_FULL;
			}
		}
		else
		{
			pfifo_1->error |= FIFO_DMA_ERROR_RX_POINT_NULL;
		}

//        DMA_SetCurrDataCounter(DMA2_Stream5, USART1_IMU_RX_LEN); //���ö�Ӧ�� DMA �������������������С
//        DMA_Cmd(DMA2_Stream5, ENABLE);                           //����DMA
    }
}

/**
  * @brief          ң�����ݵĽ���
  * @param[in]      none
  * @retval         none
  * @attention      �жϷ�����
  */
void DMA2_Stream5_IRQHandler(void)
{
    //	//�����ٽ�Σ��ٽ�ο���Ƕ��
    //	ulReturn = taskENTER_CRITICAL_FROM_ISR();

    //�ж��Ƿ�ΪDMA��������ж�
    if (DMA_GetFlagStatus(DMA2_Stream5, DMA_IT_TCIF5) == SET)
    {
        DMA_ClearFlag(DMA2_Stream5, DMA_IT_TCIF5);
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);

        if (rc_callback != NULL)
            rc_callback(RC_Rx);
    }

    //�˳��ٽ��
    //	taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}

/**
  * @brief          ��������ж�
  * @param[in]      void
  * @retval         void
  */
/*
 * ST�ٷ�����APPNOTEָ���ģ�����UARTû��RS485���ܵĵ�Ƭ���ͺŶ��ԣ���
 * 1������DMAǰ���ȹر�UART��������жϣ��������������жϱ�־��
 * 2����DMA��������жϺ����У�����UART��������жϣ�
 * 3����UART��������жϺ����У��л�RS485Ϊ����̬��
*/
void DMA2_Stream7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);
        DMA_Cmd(DMA2_Stream7, DISABLE); //��ʹ�ܴ���
		//while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		//DMA2_Stream7_working = 0;
    }
}

/**
  * @brief          ����һ+DMA ����
  * @param[in]      *data
  * @param[in]      len
  * @retval         void
  */
uint32_t usart1_dma_send(uint8_t *data, uint16_t len)
{	
	uint32_t result = fifo_write_buff(&fifo_usart_tx_1, data, len); //�����ݷ���ѭ��������
	
    //result != 0 ˵���������ݳɹ� DMA2_Stream6_working == 0 ˵������������û������
    if (DMA2_Stream7_working == 0 && result != 0)
    {
        len = fifo_read_buff(&fifo_usart_tx_1, Usart1_IMU_Tx, USART1_IMU_TX_LEN); //��ѭ����������ȡ����
		
        DMA_SetCurrDataCounter(DMA2_Stream7, len); //�趨���䳤��
        DMA_Cmd(DMA2_Stream7, ENABLE); //ʹ�ܴ���
		
        DMA2_Stream7_working = 1;
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
