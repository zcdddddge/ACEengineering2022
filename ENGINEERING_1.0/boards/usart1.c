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
	/* 遥控器 */
    if (usart1_mode == 1)
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        DMA_InitTypeDef DMA_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure; //结构体声明

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //时钟使能

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStructure); //复用io口配置

        USART_DeInit(USART1);
        USART_InitStructure.USART_BaudRate = 100000; //遥控接收波特率
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_Mode = USART_Mode_Rx; //接收
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART1, &USART_InitStructure);

		USART_ITConfig(USART1, USART_IT_IDLE, DISABLE); //接收遥控器的时候不要串口空闲中断
        USART_Cmd(USART1, ENABLE);                     //使能USART1
        USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); //使能串口DMA接收

        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = usart_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure); //中断优先级

        DMA_Cmd(DMA2_Stream5, DISABLE); //关闭DMA
        while (DMA2_Stream5->CR & DMA_SxCR_EN); //等待 DMA 可配置

        DMA_DeInit(DMA2_Stream5); //重置为缺省值
        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   //源地址
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RC_Rx;                //目的地址
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //数据传输方向为外设到内存
        DMA_InitStructure.DMA_BufferSize = RC_RX_LEN;                           //设置数据的缓冲大小
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为8位
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         //工作在循环缓存模式
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream5, &DMA_InitStructure);

        DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE); //dma传输完成产生中断
        DMA_Cmd(DMA2_Stream5, ENABLE); //开启DMA
    }
	/* 陀螺仪 */
    else if (usart1_mode == 2)
    {
        /* -------------- Enable Module Clock Source ----------------------------*/
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能GPIOA时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                      //使能USART1时钟

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //GPIOA9复用为USART1
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10复用为USART1

        /* -------------- Configure GPIO ---------------------------------------*/
        {
            GPIO_InitTypeDef GPIO_InitStruct;
            NVIC_InitTypeDef NVIC_InitStructure;
            USART_InitTypeDef USART1_InitStruct;

            //USART1端口配置
            GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
            GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;            //复用功能
            GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;       //速度50MHz
            GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
            GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
            GPIO_Init(GPIOA, &GPIO_InitStruct);                  //初始化PA9，PA10

            //USART1 初始化设置
            USART_DeInit(USART1);
            USART1_InitStruct.USART_BaudRate = 115200;                                    //波特率设置
            USART1_InitStruct.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
            USART1_InitStruct.USART_StopBits = USART_StopBits_1;                          //一个停止位
            USART1_InitStruct.USART_Parity = USART_Parity_No;                             //无奇偶校验位
            USART1_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
            USART1_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式
            USART_Init(USART1, &USART1_InitStruct);                                       //初始化串口1

            USART_Cmd(USART1, ENABLE); //使能串口1
            USART_ClearFlag(USART1, USART_FLAG_TC);

            //Usart1 NVIC 配置
            NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                  //串口1中断通道
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
            NVIC_Init(&NVIC_InitStructure);                 //根据指定的参数初始化VIC寄存器
			
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启相关中断
			USART_ITConfig(USART1, USART_IT_TC, DISABLE);
            USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
        }

        /* -------------- Configure DMA -----------------------------------------*/
		/* 发送 */
        {
            DMA_InitTypeDef DMA_InitStruct;
			NVIC_InitTypeDef NVIC_InitStructure;
			
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

            //发送数据
			DMA_Cmd(DMA2_Stream7, DISABLE); //关闭DMA
            DMA_DeInit(DMA2_Stream7); //重置为缺省值
            DMA_InitStruct.DMA_Channel = DMA_Channel_4;
            DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   //源地址
            DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart1_IMU_Tx);      //目的地址
            DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //数据传输方向为外设到内存
            DMA_InitStruct.DMA_BufferSize = USART1_IMU_TX_LEN;                   //设置数据的缓冲大小
            DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址不变
            DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //内存缓冲区地址自加
            DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //8位字节传输
            DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
            DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;                           //工作在正常模式
            DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;                 //最高优先级
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
			
			DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);  //开启发送完成中断
            DMA_Cmd(DMA2_Stream7, DISABLE);
            USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); //使能串口的DMA发送
			
			if (fifo_init(&fifo_usart_tx_1, Usart1_IMU_Tx_Buffer, USART1_IMU_TX_LEN) == -1)
			{
				// 必须 2 的幂次方
			}
        }
		// 接收数据
		{
			DMA_InitTypeDef DMA_InitStruct;
			
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			
			DMA_Cmd(DMA2_Stream5, DISABLE); //关闭DMA
			while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE)
				;
            DMA_DeInit(DMA2_Stream5); //重置为缺省值
            DMA_InitStruct.DMA_Channel = DMA_Channel_4;
            DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);   //源地址
            DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(Usart1_IMU_Rx);      //目的地址
            DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;                 //数据传输方向为外设到内存
            DMA_InitStruct.DMA_BufferSize = USART1_IMU_RX_LEN;                   //设置数据的缓冲大小
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
            DMA_Init(DMA2_Stream5, &DMA_InitStruct);
			
            DMA_Cmd(DMA2_Stream5, ENABLE);
            USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	

			if (fifo_init(pfifo_1, Usart1_IMU_Rx, USART1_IMU_RX_LEN) == -1)
			{
				// 必须 2 的幂次方
			}
		}
    }
}

void USART1_IRQHandler(void) // 接收数据中断
{
    __IO uint8_t Len = 0;
	
	//发送完成中断
	/*
	 * DMA中断时，只表示需要传送的所有数据字节全部传送到串口的发送数据寄存器中了。
	 * 此时串口实际上还有2个字节并未发送完成，数据寄存器和移位寄存器中的2个字节还需要发送，并不能关闭串口发送。
	 * 同理，如果是485切换方向，必须要等到发送完成，也就是移位寄存器发送完成-TC标志置位。
	*/
    if (USART_GetITStatus(USART1, USART_IT_TC) == SET)
    {
		USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
		DMA2_Stream7_working = 0;
    }
	//总线空闲中断
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) //触发中断标志位
    {
        //DMA_Cmd(DMA2_Stream5, DISABLE); //关闭DMA,防止处理期间有数据
		//DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
        //while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);

        Len = USART1->SR; //清除RXNE标志
        Len = USART1->DR; //清USART_IT_IDLE标志  
        Len = DMA_GetCurrDataCounter(DMA2_Stream5); //获取当前剩余数据量大小的函数

		if (pfifo_1 != 0)
		{
			// Len为当前接收索引
			pfifo_1->in += ((pfifo_1->last_cnt - Len) & (pfifo_1->size - 1)); //更新in
			pfifo_1->last_cnt = Len;

			if ((pfifo_1->in - pfifo_1->out) > pfifo_1->size)
			{
				pfifo_1->out = pfifo_1->in; // 清空缓存，注意赋值顺序，pfifo->in = pfifo->out 是错误的
				pfifo_1->error |= FIFO_DMA_ERROR_RX_FULL;
			}
		}
		else
		{
			pfifo_1->error |= FIFO_DMA_ERROR_RX_POINT_NULL;
		}

//        DMA_SetCurrDataCounter(DMA2_Stream5, USART1_IMU_RX_LEN); //设置对应的 DMA 数据流传输的数据量大小
//        DMA_Cmd(DMA2_Stream5, ENABLE);                           //开启DMA
    }
}

/**
  * @brief          遥控数据的接收
  * @param[in]      none
  * @retval         none
  * @attention      中断服务函数
  */
void DMA2_Stream5_IRQHandler(void)
{
    //	//进入临界段，临界段可以嵌套
    //	ulReturn = taskENTER_CRITICAL_FROM_ISR();

    //判断是否为DMA发送完成中断
    if (DMA_GetFlagStatus(DMA2_Stream5, DMA_IT_TCIF5) == SET)
    {
        DMA_ClearFlag(DMA2_Stream5, DMA_IT_TCIF5);
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);

        if (rc_callback != NULL)
            rc_callback(RC_Rx);
    }

    //退出临界段
    //	taskEXIT_CRITICAL_FROM_ISR(ulReturn);
}

/**
  * @brief          发送完成中断
  * @param[in]      void
  * @retval         void
  */
/*
 * ST官方都有APPNOTE指导的（对于UART没有RS485功能的单片机型号而言）：
 * 1、启动DMA前，先关闭UART发送完成中断，并清除发送完成中断标志；
 * 2、在DMA传输完成中断函数中，开启UART发送完成中断；
 * 3、在UART发送完成中断函数中，切换RS485为接收态；
*/
void DMA2_Stream7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
    {
        DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);
        DMA_Cmd(DMA2_Stream7, DISABLE); //不使能传输
		//while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		//DMA2_Stream7_working = 0;
    }
}

/**
  * @brief          串口一+DMA 发送
  * @param[in]      *data
  * @param[in]      len
  * @retval         void
  */
uint32_t usart1_dma_send(uint8_t *data, uint16_t len)
{	
	uint32_t result = fifo_write_buff(&fifo_usart_tx_1, data, len); //将数据放入循环缓冲区
	
    //result != 0 说明放入数据成功 DMA2_Stream6_working == 0 说明缓冲区里面没有数据
    if (DMA2_Stream7_working == 0 && result != 0)
    {
        len = fifo_read_buff(&fifo_usart_tx_1, Usart1_IMU_Tx, USART1_IMU_TX_LEN); //从循环缓冲区获取数据
		
        DMA_SetCurrDataCounter(DMA2_Stream7, len); //设定传输长度
        DMA_Cmd(DMA2_Stream7, ENABLE); //使能传输
		
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
