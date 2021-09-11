#include "uart4.h"
#include "parameter.h"
#include "fifo_buff.h"


#define UART4_RX_LEN 512
#define UART4_TX_LEN 256
uint8_t Uart4_Rx[UART4_RX_LEN] = {0};
uint8_t Uart4_Tx[UART4_TX_LEN] = {0};
uint8_t Uart4_Tx_Buffer[UART4_TX_LEN] = {0};

fifo_rx_def fifo_uart_rx_4;
fifo_rx_def *pfifo_4 = &fifo_uart_rx_4;
fifo_rx_def fifo_uart_tx_4;

uint8_t DMA1_Stream4_working = 0;

/*************************************************************************************************
*名称:	USART4_GYRO_INIT_PC11_RX_PC10_TX
*功能:	USART4初始化配置
*形参: 	无
*返回:	无
*说明:	陀螺仪
*************************************************************************************************/
void UART4_GYRO_INIT_PC11_RX_PC10_TX(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStruct; 
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	USART_DeInit(UART4);
	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART4,&USART_InitStruct);
	
	NVIC_InitStructure.NVIC_IRQChannel						=	UART4_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	6;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE); 
	USART_Cmd(UART4,ENABLE);

	DMA_DeInit(DMA1_Stream2);					
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);			
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(PYR_rx_buffer);			
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;				
	DMA_InitStruct.DMA_BufferSize = 64;							
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;							
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;												
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;							
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2,&DMA_InitStruct);
	DMA_Cmd(DMA1_Stream2,ENABLE);
	
	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);	
}


void uart4_init(uint8_t usart_NVIC)
{
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_InitTypeDef USART_InitStruct;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOC, &GPIO_InitStruct);

		USART_DeInit(UART4);
		USART_InitStruct.USART_BaudRate = 115200;
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART_InitStruct.USART_StopBits = USART_StopBits_1;
		USART_InitStruct.USART_Parity = USART_Parity_No;
		USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(UART4, &USART_InitStruct);

		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&NVIC_InitStructure);
		
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //开启相关中断
		USART_ITConfig(UART4, USART_IT_TC, DISABLE);
		USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
		USART_Cmd(UART4, ENABLE);
	}
	/* DMA发送 */
	{
		DMA_InitTypeDef DMA_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		
        //发送数据
        DMA_DeInit(DMA1_Stream4); //重置为缺省值
		DMA_StructInit(&DMA_InitStruct);
        DMA_InitStruct.DMA_Channel = DMA_Channel_4;
        DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (UART4->DR);    //源地址
        DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Uart4_Tx);           //目的地址
        DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //数据传输方向为外设到内存
        DMA_InitStruct.DMA_BufferSize = UART4_TX_LEN;                        //设置数据的缓冲大小
        DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
        DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;                           //正常工作模式
        DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
        DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		//DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
		//DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		//DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		//DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream4, &DMA_InitStruct);
		
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
		DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);  //开启发送完成中断
        DMA_Cmd(DMA1_Stream4, DISABLE);
        USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); //使能串口的DMA发送
		
		if (fifo_init(&fifo_uart_tx_4, Uart4_Tx_Buffer, UART4_TX_LEN) == -1)
		{
			// 必须 2 的幂次方
		}
	}
	/* DMA接收 */
    {
		DMA_InitTypeDef DMA_InitStruct;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		
        //接收数据
		DMA_DeInit(DMA1_Stream2);
		DMA_InitStruct.DMA_Channel = DMA_Channel_4;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (UART4->DR);
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Uart4_Rx);
		DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStruct.DMA_BufferSize = UART4_RX_LEN;
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
//		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;
//		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//		DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream2, &DMA_InitStruct);
		DMA_Cmd(DMA1_Stream2, ENABLE);
        USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
		
		if (fifo_init(pfifo_4, Uart4_Rx, UART4_RX_LEN) == -1)
		{
			// 必须 2 的幂次方
		}
    }
}



/**
  * @brief          发送完成中断
  * @param[in]      void
  * @retval         void
  */
void DMA1_Stream4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4) != RESET)
    {
        DMA_ClearFlag(DMA1_Stream4, DMA_IT_TCIF4);
        DMA_Cmd(DMA1_Stream4, DISABLE); //不使能传输
		USART_ITConfig(UART4, USART_IT_TC, ENABLE);
    }
}

/**
  * @brief          串口四+DMA 发送
  * @param[in]      void
  * @retval         void
  */
uint32_t uart4_dma_send(uint8_t *data, uint16_t len)
{	
    uint32_t result = fifo_write_buff(&fifo_uart_tx_4, data, len); //将数据放入循环缓冲区
	
    //result != 0 说明放入数据成功 DMA2_Stream6_working == 0 说明缓冲区里面没有数据
    if (DMA1_Stream4_working == 0)
    {
        len = fifo_read_buff(&fifo_uart_tx_4, Uart4_Tx, UART4_TX_LEN); //从循环缓冲区获取数据
		
        DMA_SetCurrDataCounter(DMA1_Stream4, len); //设定传输长度
        DMA_Cmd(DMA1_Stream4, ENABLE); //使能传输
		
        DMA1_Stream4_working = 1;
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


