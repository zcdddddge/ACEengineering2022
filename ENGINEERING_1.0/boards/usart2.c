#include "usart2.h"
#include "parameter.h"
#include "fifo_buff.h"

#define USART2_RX_LEN 256
#define USART2_TX_LEN 256
uint8_t Usart2_Rx[USART2_RX_LEN] = {0};
uint8_t Usart2_Tx[USART2_TX_LEN] = {0};

fifo_rx_def fifo_usart_rx_2;
fifo_rx_def *pfifo_2 = &fifo_usart_rx_2;



//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while ((USART2->SR & 0X40) == 0)
        ; //ѭ������,ֱ���������

    USART2->DR = (uint8_t)ch;
    return ch;
}
#endif

void usart2_init(uint8_t usart_NVIC)
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);    //ʹ��USART3ʱ��
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //GPIOD8����ΪUSART3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); //GPIOD9����ΪUSART3

    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        NVIC_InitTypeDef NVIC_InitStructure;
        USART_InitTypeDef USART2_InitStruct;

        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOD, &GPIO_InitStruct);

        USART_DeInit(USART2);
        USART2_InitStruct.USART_BaudRate = 115200;
        USART2_InitStruct.USART_WordLength = USART_WordLength_8b;
        USART2_InitStruct.USART_StopBits = USART_StopBits_1;
        USART2_InitStruct.USART_Parity = USART_Parity_No; //����żУ��
        USART2_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART2_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART2, &USART2_InitStruct);

        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //DMA1_Stream1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_Init(&NVIC_InitStructure);
		
		
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //��������ж�
		USART_ITConfig(USART2, USART_IT_TC, DISABLE);
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART2, ENABLE);
    }

    /* -------------- Configure DMA -----------------------------------------*/
    {
        DMA_InitTypeDef DMA_InitStruct;

        //��������
        DMA_DeInit(DMA1_Stream6); //����Ϊȱʡֵ
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);   //Դ��ַ
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart2_Tx);          //Ŀ�ĵ�ַ
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStruct.DMA_BufferSize = USART2_TX_LEN;                       //�������ݵĻ����С
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
        DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;                         //������ѭ������ģʽ
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream6, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream6, DISABLE);
        USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); //ʹ�ܴ���2��DMA����

        // ��������
        DMA_DeInit(DMA1_Stream5); //����Ϊȱʡֵ
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);   //Դ��ַ
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart2_Rx);          //Ŀ�ĵ�ַ
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStruct.DMA_BufferSize = USART2_RX_LEN;                       //�������ݵĻ����С
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ����
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�ڴ滺������ַ�Լ�
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8λ�ֽڴ���
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ��Ϊ8λ
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //������ѭ������ģʽ
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //������ȼ�
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream5, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream5, ENABLE);
        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    }

    fifo_init(&fifo_usart_rx_2, Usart2_Rx, USART2_RX_LEN);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); //���������ж�
}

void USART2_IRQHandler(void) // ���������ж�
{
    volatile uint32_t Len = 0;
	
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) //�����жϱ�־λ
    {
        Len = USART2->SR;
        Len = USART2->DR;

        Len = DMA_GetCurrDataCounter(DMA1_Stream5);

		if (pfifo_2 != 0)
		{
			// LenΪ��ǰ��������
			pfifo_2->in += ((pfifo_2->last_cnt - Len) & (pfifo_2->size - 1)); //����in
			pfifo_2->last_cnt = Len;

			if ((pfifo_2->in - pfifo_2->out) > pfifo_2->size)
			{
				pfifo_2->out = pfifo_2->in; // ��ջ��棬ע�⸳ֵ˳��pfifo->in = pfifo->out �Ǵ����
				pfifo_2->error |= FIFO_DMA_ERROR_RX_FULL;
			}
		}
		else
		{
			pfifo_2->error |= FIFO_DMA_ERROR_RX_POINT_NULL;
		}
    }
}
