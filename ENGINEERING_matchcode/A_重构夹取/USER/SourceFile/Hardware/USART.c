/*************************************************************************************************
模块名称:	USART
实现功能:	实现USART的硬件配置
备		注:	无
-------------------------------------------------------------------------------------------------
日期:	2020-1-5
版本:	1.0
功能:	实现USART1 USART2 USART3 USART6的初始化配置
说明:	无
*************************************************************************************************/
#include "USART.h"

/*变量定义*/
volatile unsigned char dbus_rx_buffer[25];		
volatile unsigned char gyro_rx_buffer[25];
volatile unsigned char referee_rx_buffer[128];
volatile unsigned char vision_rx_buffer[32];	
volatile unsigned char vL53L0_rx_buffer[128];

/*************************************************************************************************
*名称:	USART1_RC_INIT_PB7_RX
*功能:	USART1初始化配置
*形参: 	无
*返回:	无
*说明:	用于接收遥控值数据
*************************************************************************************************/
void USART1_RC_INIT_PB7_RX(void) 															
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* config USART1 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB| RCC_AHB1Periph_DMA2 , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_USART1);
	
	/*遥控Pin口 复用PB7*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;								                                                                                                                                                                                                                                                                                                       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);
		
	/* USART1 mode config */
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 100000;							 											//SBUS 100K baudrate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;  						        						//接收模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流
	USART_Init(USART1,&USART_InitStructure);

	USART_Cmd(USART1,ENABLE);												 
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);							

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA2_Stream5, DISABLE);						
	while (DMA2_Stream5->CR & DMA_SxCR_EN);

	DMA_DeInit(DMA2_Stream5);												 														//重置为缺省值
	DMA_InitStructure.DMA_Channel= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);		  		//源地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dbus_rx_buffer;		 				//目的地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					 						//数据传输方向为外设到内存
	DMA_InitStructure.DMA_BufferSize = 18;									 										//设置数据的缓冲大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		 				//外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					 						//内存缓冲区地址自加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	 		//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		 				//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							 								//工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					 						//最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5,&DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream5,ENABLE);
}




/*************************************************************************************************
*名称:	USART1_RC_INIT_PA10_RX
*功能:	USART1初始化配置
*形参: 	无
*返回:	无
*说明:	用于接收遥控值数据
*************************************************************************************************/
void USART1_RC_INIT_PA10_RX(void) 															
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA| RCC_AHB1Periph_DMA2 , ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);    
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;		                                                                                                                                                                                                                                                                                       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOA,&GPIO_InitStructure);       
	
	/* USART1 mode config */
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 100000;							 											//SBUS 100K baudrate
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;  						        						//接收模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流
	USART_Init(USART1,&USART_InitStructure);

	USART_Cmd(USART1,ENABLE);												 
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);							

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_Cmd(DMA2_Stream5, DISABLE);						
	while (DMA2_Stream5->CR & DMA_SxCR_EN);

	DMA_DeInit(DMA2_Stream5);												 														//重置为缺省值
	DMA_InitStructure.DMA_Channel= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);		  		//源地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dbus_rx_buffer;		 				//目的地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					 						//数据传输方向为外设到内存
	DMA_InitStructure.DMA_BufferSize = 18;									 										//设置数据的缓冲大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		 				//外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					 						//内存缓冲区地址自加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	 		//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		 				//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;							 								//工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					 						//最高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5,&DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream5,ENABLE);
}




/*************************************************************************************************
*名称:	USART6_VL53L0_INIT_PC7_RX_PC6_TX
*功能:	USART6初始化配置
*形参: 	无
*返回:	无
*说明:	用于接收测距传感器 
*************************************************************************************************/
void USART6_VL53L0_INIT_PC7_RX_PC6_TX(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6);
	
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_InitTypeDef USART6_InitStruct;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		USART_DeInit(USART6);
		USART6_InitStruct.USART_BaudRate = 115200;
		USART6_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART6_InitStruct.USART_StopBits = USART_StopBits_1;
		USART6_InitStruct.USART_Parity = USART_Parity_No; 
		USART6_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART6,&USART6_InitStruct);
		
		NVIC_InitStructure.NVIC_IRQChannel						=	USART6_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	3;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE); 
		USART_Cmd(USART6,ENABLE);
	}
	
	  /* 		 Configure DMA 		*/
	{
		DMA_InitTypeDef DMA_InitStruct;
		DMA_DeInit(DMA2_Stream1);																							//重置为缺省值
		DMA_InitStruct.DMA_Channel = DMA_Channel_5;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);			//源地址
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(vL53L0_rx_buffer);		//目的地址
		DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;									//数据传输方向为外设到内存
		DMA_InitStruct.DMA_BufferSize = 128;																	//设置数据的缓冲大小
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//外设地址不变
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;									//内存缓冲区地址自加
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//8位字节传输
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				  //数据宽度为8位
		DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;													//工作在循环缓存模式
		DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;									//最高优先级
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream1,&DMA_InitStruct);
		DMA_Cmd(DMA2_Stream1,ENABLE);
		
		USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);		
	}
}





/*************************************************************************************************
*名称:	USART6_REFEREE_INIT_PG9_RX_PG14_TX
*功能:	USART6初始化配置
*形参: 	&usart6_rx_buf->数据存储的地址	usart6_buf_num->数据缓存区的大小--256
*返回:	无
*说明:	用于接收裁判系统返回的数据
*************************************************************************************************/
void USART6_REFEREE_INIT_PG9_RX_PG14_TX(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14, GPIO_AF_USART6);
	
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_InitTypeDef USART6_InitStruct;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_14;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOG, &GPIO_InitStruct);
		
		USART_DeInit(USART6);
		USART6_InitStruct.USART_BaudRate = 115200;
		USART6_InitStruct.USART_WordLength = USART_WordLength_8b;
		USART6_InitStruct.USART_StopBits = USART_StopBits_1;
		USART6_InitStruct.USART_Parity = USART_Parity_No; 
		USART6_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		USART6_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART6,&USART6_InitStruct);
		
		NVIC_InitStructure.NVIC_IRQChannel						=	USART6_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	3;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE); 
		USART_Cmd(USART6,ENABLE);
	}
	
	  /* 		 Configure DMA 		*/
	{
		DMA_InitTypeDef DMA_InitStruct;
		DMA_DeInit(DMA2_Stream1);																							//重置为缺省值
		DMA_InitStruct.DMA_Channel = DMA_Channel_5;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);			//源地址
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(referee_rx_buffer);		//目的地址
		DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;									//数据传输方向为外设到内存
		DMA_InitStruct.DMA_BufferSize = 128;																	//设置数据的缓冲大小
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//外设地址不变
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;									//内存缓冲区地址自加
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//8位字节传输
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				  //数据宽度为8位
		DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;													//工作在循环缓存模式
		DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;									//最高优先级
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream1,&DMA_InitStruct);
		DMA_Cmd(DMA2_Stream1,ENABLE);
		
		USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);		
	}
}



/*************************************************************************************************
*名称:	USART3_VISION_INIT_PD9_RX_PD8_TX
*功能:	USART3初始化配置
*形参: 	无
*返回:	无
*说明:	视觉
*************************************************************************************************/
void USART3_VISION_INIT_PD9_RX_PD8_TX(void)
{
/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);									//使能USART3时钟
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); 							//GPIOD8复用为USART3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 							//GPIOD9复用为USART3
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
		USART3_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		USART3_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&USART3_InitStruct);
		
		NVIC_InitStructure.NVIC_IRQChannel						=	USART3_IRQn;		//DMA1_Stream1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); 
		USART_Cmd(USART3,ENABLE);
}
	/* -------------- Configure DMA -----------------------------------------*/
	{
		// 接收数据
		DMA_InitTypeDef DMA_InitStruct;
		DMA_DeInit(DMA1_Stream1);					//重置为缺省值
		DMA_InitStruct.DMA_Channel = DMA_Channel_4;
		DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);			//源地址
		DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(vision_rx_buffer);		//目的地址
		DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;									//数据传输方向为外设到内存
		DMA_InitStruct.DMA_BufferSize = 32;																		//设置数据的缓冲大小
		DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//外设地址不变
		DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;									//内存缓冲区地址自加
		DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//8位字节传输
		DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
		DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;													//工作在循环缓存模式
		DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;									//最高优先级
		DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		DMA_InitStruct.DMA_MemoryBurst = DMA_Mode_Normal;
		DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream1,&DMA_InitStruct);
		DMA_Cmd(DMA1_Stream1,ENABLE);
		
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);	
	}
}



/*************************************************************************************************
*名称:	USART2_GYRO_INIT_PD6_RX_PD5_TX
*功能:	USART2初始化配置
*形参: 	无
*返回:	无
*说明:	陀螺仪
*************************************************************************************************/
void USART2_GYRO_INIT_PD6_RX_PD5_TX(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStruct; 
	DMA_InitTypeDef DMA_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	USART_DeInit(USART2);
	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&USART_InitStruct);
	
	NVIC_InitStructure.NVIC_IRQChannel						=	USART2_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	5;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); 
	USART_Cmd(USART2,ENABLE);

	DMA_DeInit(DMA1_Stream5);					
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);			
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)(gyro_rx_buffer);			
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;				
	DMA_InitStruct.DMA_BufferSize = 25;							
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
	DMA_Init(DMA1_Stream5,&DMA_InitStruct);
	DMA_Cmd(DMA1_Stream5,ENABLE);
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);	
}


/*************************************************************************************************
*名称:	USART_Disable
*功能:	USART失能
*形参: 	USARTx
*返回:	无
*说明:	失能某串口
*************************************************************************************************/
void USART_DISABLE(USART_TypeDef* USARTx)
{
	USART_Cmd(USARTx, DISABLE);
}

