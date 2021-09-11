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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    //使能USART3时钟
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); //GPIOD8复用为USART3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); //GPIOD9复用为USART3
	
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
        USART3_InitStruct.USART_Parity = USART_Parity_No; //无奇偶校验
        USART3_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART3_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART3, &USART3_InitStruct);

        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //DMA1_Stream1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = usart_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_Init(&NVIC_InitStructure);
		
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //开启相关中断
		USART_ITConfig(USART3, USART_IT_TC, DISABLE);
		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
        USART_Cmd(USART3, ENABLE);
    }
    /* -------------- Configure DMA -----------------------------------------*/
    {
        //发送数据
        DMA_InitTypeDef DMA_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		
		DMA_Cmd(DMA1_Stream3, DISABLE); //关闭DMA
        DMA_DeInit(DMA1_Stream3); //重置为缺省值
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);   //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart3_Tx);          //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = USART3_TX_LEN;                       //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;                         //工作在循环缓存模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
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
	
		DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);  //开启发送完成中断
        DMA_Cmd(DMA1_Stream3, DISABLE);
        USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE); //使能串口3的DMA发送

		if (fifo_init(&fifo_usart_tx_3, Usart3_Tx_Buffer, USART3_TX_LEN) == -1)
		{
			// 必须 2 的幂次方
		}
    }
	{
		DMA_InitTypeDef DMA_InitStruct;
		
        // 接收数据
        DMA_DeInit(DMA1_Stream1); //重置为缺省值
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);   //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart3_Rx);          //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = USART3_RX_LEN;                       //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                         //工作在循环缓存模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal; //DMA_MemoryBurst_Single;//
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream1, &DMA_InitStruct);
        DMA_Cmd(DMA1_Stream1, ENABLE);
        USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);		
		
		if (fifo_init(pfifo_3, Usart3_Rx, USART3_RX_LEN) == -1)
		{
			// 必须 2 的幂次方
		}
	}
}

void USART3_IRQHandler(void)
{
    __IO uint8_t Len = 0;

	//发送完成中断
	/*
	 * DMA中断时，只表示需要传送的所有数据字节全部传送到串口的发送数据寄存器中了。
	 * 此时串口实际上还有2个字节并未发送完成，数据寄存器和移位寄存器中的2个字节还需要发送，并不能关闭串口发送。
	 * 同理，如果是485切换方向，必须要等到发送完成，也就是移位寄存器发送完成-TC标志置位。
	*/
    if (USART_GetITStatus(USART3, USART_IT_TC) == SET)
    {
		USART_ClearITPendingBit(USART3, USART_IT_TC);
        USART_ITConfig(USART3, USART_IT_TC, DISABLE);
		DMA1_Stream3_working = 0;
    }
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) //触发中断标志位
    {
        Len = USART3->SR;
        Len = USART3->DR;
        Len = DMA_GetCurrDataCounter(DMA1_Stream1);

		if (pfifo_3 != 0)
		{
			// Len为当前接收索引
			pfifo_3->in += ((pfifo_3->last_cnt - Len) & (pfifo_3->size - 1)); //更新in
			pfifo_3->last_cnt = Len;

			if ((pfifo_3->in - pfifo_3->out) > pfifo_3->size)
			{
				pfifo_3->out = pfifo_3->in; // 清空缓存，注意赋值顺序，pfifo->in = pfifo->out 是错误的
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
  * @brief          发送完成中断
  * @param[in]      void
  * @retval         void
  */
void DMA1_Stream3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)
    {
        DMA_ClearFlag(DMA1_Stream3, DMA_IT_TCIF3);
        DMA_Cmd(DMA1_Stream3, DISABLE); //不使能传输
		USART_ITConfig(USART3, USART_IT_TC, ENABLE);
    }
}

/**
  * @brief          串口四+DMA 发送
  * @param[in]      void
  * @retval         void
  */
uint32_t usart3_dma_send(uint8_t *data, uint16_t len)
{	
    uint32_t result = fifo_write_buff(&fifo_usart_tx_3, data, len); //将数据放入循环缓冲区
	
    //result != 0 说明放入数据成功 DMA2_Stream6_working == 0 说明缓冲区里面没有数据
    if (DMA1_Stream3_working == 0)
    {
        len = fifo_read_buff(&fifo_usart_tx_3, Usart3_Tx, USART3_TX_LEN); //从循环缓冲区获取数据
		
        DMA_SetCurrDataCounter(DMA1_Stream3, len); //设定传输长度
        DMA_Cmd(DMA1_Stream3, ENABLE); //使能传输
		
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


