#include "CAN.h"

/*************************************************************************************************
*名称:	CAN1_Init_GPIOA_IO11_IO12
*功能:	CAN1初始化配置
*形参: 	mode->CAN_Mode_Normal CAN_Mode_LoopBack CAN_Mode_Silent CAN_Mode_Silent_LoopBack
*返回:	无
*说明:	CAN是挂载在APB1时钟(0.25个主时钟)
				CAN BaudRate = APB1/(tsjw+tbs2+tbs1)/brp
*************************************************************************************************/
void CAN1_Init_GPIOA_IO11_IO12(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
		NVIC_InitTypeDef       NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);					//打开GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);			//配置引脚复用功能
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;  		//CAN1_TX(PD1) CAN1_RX(PD0)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									//复用推挽输出
		GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;				
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    
    CAN_DeInit(CAN1);                        						//将外围寄存器初始化到它们的默认重置值
    CAN_StructInit(&CAN_InitStructure);      						//用它的默认值填充每个CAN_InitStructure成员
    /************CAN总线的配置*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE;								//非时间触发通信模式
    CAN_InitStructure.CAN_ABOM = DISABLE;								//软件自动离线管理模式
    CAN_InitStructure.CAN_AWUM = DISABLE;								//自动唤醒模式，睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART = DISABLE;								//非自动重传输模式，禁止报文自动传送 
    CAN_InitStructure.CAN_RFLM = DISABLE;								//接收FIFO锁定模式，报文不锁定,新的覆盖旧的 
    CAN_InitStructure.CAN_TXFP = ENABLE;								//发送FIFO优先迹，优先级由报文标识符决定 
    CAN_InitStructure.CAN_Mode = mode;					
		CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;   
    CAN_Init(CAN1, &CAN_InitStructure);
		/******CAN总线的过滤配置(接收配置)********/
    CAN_FilterInitStructure.CAN_FilterNumber=0;     						//过滤器 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;					//the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);										//滤波器初始化   

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);												//接收中断
}



/*************************************************************************************************
*名称:	CAN1_Init_GPIOD_IO0_IO1
*功能:	CAN1初始化配置
*形参: 	mode->CAN_Mode_Normal CAN_Mode_LoopBack CAN_Mode_Silent CAN_Mode_Silent_LoopBack
*返回:	无
*说明:	CAN是挂载在APB1时钟(0.25个主时钟)
				CAN BaudRate = APB1/(tsjw+tbs2+tbs1)/brp
*************************************************************************************************/
void CAN1_Init_GPIOD_IO0_IO1(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
		NVIC_InitTypeDef       NVIC_InitStructure;

	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);		 //打开GPIO时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);     //打开can1时钟
		
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);	 //配置引脚复用功能
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);  //配置引脚复用功能

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						
		GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;				
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;   	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    
    CAN_DeInit(CAN1);                        						//将外围寄存器初始化到它们的默认重置值
    CAN_StructInit(&CAN_InitStructure);      						//用它的默认值填充每个CAN_InitStructure成员
    /************CAN总线的配置*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE;								//非时间触发通信模式
    CAN_InitStructure.CAN_ABOM = ENABLE;								//软件自动离线管理模式
    CAN_InitStructure.CAN_AWUM = DISABLE;								//自动唤醒模式，睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART = DISABLE;								//非自动重传输模式，禁止报文自动传送 
    CAN_InitStructure.CAN_RFLM = DISABLE;								//接收FIFO锁定模式，报文不锁定,新的覆盖旧的 
    CAN_InitStructure.CAN_TXFP = ENABLE;								//发送FIFO优先迹，优先级由报文标识符决定 
    CAN_InitStructure.CAN_Mode = mode;					
		CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;   
    CAN_Init(CAN1, &CAN_InitStructure);
		/******CAN总线的过滤配置(接收配置)********/
    CAN_FilterInitStructure.CAN_FilterNumber=0;     						//过滤器 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;					//the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);										//滤波器初始化   

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);												//接收中断
}


/*************************************************************************************************
*名称:	CAN2_Init_GPIOB_IO12_IO13
*功能:	CAN2初始化配置
*形参: 	mode->CAN_Mode_Normal CAN_Mode_LoopBack CAN_Mode_Silent CAN_Mode_Silent_LoopBack
*返回:	无
*说明:	CAN是挂载在APB1时钟(0.25个主时钟)
				CAN BaudRate = APB1/(tsjw+tbs2+tbs1)/brp
*************************************************************************************************/
void CAN2_Init_GPIOB_IO12_IO13(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
		NVIC_InitTypeDef       NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);					//打开GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);	

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);			//配置引脚复用功能
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									//复用推挽输出
    GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;					
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure); 
    
    CAN_DeInit(CAN2);                        								//将外围寄存器初始化到它们的默认重置值
    CAN_StructInit(&CAN_InitStructure);      								//用它的默认值填充每个CAN_InitStructure成员
    /************CAN总线的配置*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE;										//非时间触发通信模式
    CAN_InitStructure.CAN_ABOM = DISABLE;										//软件自动离线管理模式
    CAN_InitStructure.CAN_AWUM = DISABLE;										//自动唤醒模式，睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_InitStructure.CAN_NART = DISABLE;										//非自动重传输模式，禁止报文自动传送 
    CAN_InitStructure.CAN_RFLM = DISABLE;										//接收FIFO锁定模式，报文不锁定,新的覆盖旧的 
    CAN_InitStructure.CAN_TXFP = ENABLE;										//发送FIFO优先迹，优先级由报文标识符决定 
    CAN_InitStructure.CAN_Mode = mode;											//模式设置： mode:0,普通模式;1,回环模式;
    CAN_InitStructure.CAN_SJW  = tsjw;
    CAN_InitStructure.CAN_BS1=tbs1; 												//Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
		CAN_InitStructure.CAN_BS2=tbs2;						  						//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = brp;   							  //brp    
    CAN_Init(CAN2, &CAN_InitStructure);
		/******CAN总线的过滤配置(接收配置)********/
    CAN_FilterInitStructure.CAN_FilterNumber=14;                  	  	 //过滤器 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;    //the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);							  							//滤波器初始化   

	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);							  	  							//接收中断 
}



/*************************************************************************************************
*名称:	CAN2_Init_GPIOB_IO5_IO6
*功能:	CAN2初始化配置
*形参: 	mode->CAN_Mode_Normal CAN_Mode_LoopBack CAN_Mode_Silent CAN_Mode_Silent_LoopBack
*返回:	无
*说明:	CAN是挂载在APB1时钟(0.25个主时钟)
				CAN BaudRate = APB1/(tsjw+tbs2+tbs1)/brp
*************************************************************************************************/
void CAN2_Init_GPIOB_IO5_IO6(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode)
{
			CAN_InitTypeDef        CAN_Init2;
    CAN_FilterInitTypeDef  CAN_Filter2;
    GPIO_InitTypeDef       GPIO_Init_can2;
		NVIC_InitTypeDef       NVIC_Init_can2;                  //定义结构体

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		//打开GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);    //打开can1时钟

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);	//配置引脚复用功能
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);  //配置引脚复用功能

    GPIO_Init_can2.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init_can2.GPIO_Mode = GPIO_Mode_AF;								  //复用推挽输出
    GPIO_Init_can2.GPIO_OType=  GPIO_OType_PP;
		GPIO_Init_can2.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init_can2.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_Init_can2);
    //CAN总线io口配置 
		NVIC_Init_can2.NVIC_IRQChannel = CAN2_RX0_IRQn;					//中断配置 
		NVIC_Init_can2.NVIC_IRQChannelPreemptionPriority = 2;           //抢占优先级 1
		NVIC_Init_can2.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_Init_can2);

    CAN_DeInit(CAN2);                        //将外围寄存器初始化到它们的默认重置值
    CAN_StructInit(&CAN_Init2);      //用它的默认值填充每个CAN_InitStructure成员
    //CAN总线的配置
    CAN_Init2.CAN_TTCM = DISABLE;			      //非时间触发通信模式
    CAN_Init2.CAN_ABOM = DISABLE;			      //软件自动离线管理模式
    CAN_Init2.CAN_AWUM = DISABLE;			      //自动唤醒模式，睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
    CAN_Init2.CAN_NART = DISABLE;			      //非自动重传输模式，禁止报文自动传送 
    CAN_Init2.CAN_RFLM = DISABLE;			      //接收FIFO锁定模式，报文不锁定,新的覆盖旧的 
    CAN_Init2.CAN_TXFP = ENABLE;			      //发送FIFO优先迹，优先级由报文标识符决定 
    CAN_Init2.CAN_Mode = mode;						//模式设置： mode:0,普通模式;1,回环模式;
    CAN_Init2.CAN_SJW  = tsjw;
    CAN_Init2.CAN_BS1 = tbs1;
    CAN_Init2.CAN_BS2 = tbs2;
    CAN_Init2.CAN_Prescaler = brp;   //CAN BaudRate 42/(1+5+8)/3=1Mbps  (电机的can波特率为1m)
    CAN_Init(CAN2, &CAN_Init2);
		//CAN总线的过滤配置(接收配置)
    CAN_Filter2.CAN_FilterNumber=14;                  //过滤器 0
    CAN_Filter2.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_Filter2.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_Filter2.CAN_FilterIdHigh=0x0000;
    CAN_Filter2.CAN_FilterIdLow=0x0000;
    CAN_Filter2.CAN_FilterMaskIdHigh=0x0000;
    CAN_Filter2.CAN_FilterMaskIdLow=0x0000;
    CAN_Filter2.CAN_FilterFIFOAssignment=0;
    CAN_Filter2.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_Filter2);			//滤波器初始化   

		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);		//接收中断    启用或禁用指定的CANx中断    CAN_IT_FMP0:等待中断的FIFO 0消息	
}
