#include "TIM.h"


/*************************************************************************************************
*名称:	TIM2_Init
*功能:	TIM2定时器初始化
*形参: 	Period->u16		Prescaler->u16
*返回:	无
*说明:	无
*************************************************************************************************/
void TIM2_Init(uint16_t Period, uint16_t Prescaler)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = Period - 1;
    tim.TIM_Prescaler = Prescaler - 1;	 
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_Cmd(TIM2,ENABLE);	
}



/*************************************************************************************************
*名称:	TIM3_Init
*功能:	TIM3定时器初始化
*形参: 	Period->u16		Prescaler->u16
*返回:	无
*说明:	无
*************************************************************************************************/
void TIM3_Init(uint16_t Period, uint16_t Prescaler)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    
		tim.TIM_Prescaler = Period-1;        
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = Prescaler - 1;  
    TIM_TimeBaseInit(TIM3,&tim);
	
    nvic.NVIC_IRQChannel = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 8;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); 
    TIM_Cmd(TIM3,ENABLE);
}

