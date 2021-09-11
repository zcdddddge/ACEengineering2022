#include "time.h"

/************************************************************ TIM2 **************************************************************/
void tim2_init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 90 - 1; //1M ��ʱ��
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_TimeBaseInit(TIM2, &tim);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
    }

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

uint32_t Get_Time_Micros(void)
{
    return TIM2->CNT;
}

/************************************************************ TIM6 **************************************************************/
/*
*���ܣ���ʱ��6 ���벶��
*/
void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 90 - 1; //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000; //1ms,1000Hz
    TIM_TimeBaseInit(TIM6, &tim);

    TIM_Cmd(TIM6, ENABLE);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}

int32_t counting = 0;
static int16_t ms_count = 0;
int encoder[2] = {0, 0}; // ���α�������ֵ�����Լ����������
int volatile N = 0;      // Ȧ��
int volatile EncCnt = 0; // ������ȡֵ

void TIM6_DAC_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);

        ms_count++;

        if (ms_count == 10)
        {
            ms_count = 0;
        }
    }
}
