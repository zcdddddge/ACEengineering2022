#include "pwm.h"

void friction_wheel_init(void)
{
    /* ��C615����ֲ᣺
     *       �����ݿ����ź�Ƶ��: 500Hz  ��Ŀǰ��473Hz ��ֹ����
     *       �����ź��г�: 400~2200΢�� ��0.4ms~2.2ms��
     *       Ĭ�����PWMƵ��: 16kHz ����SNAIL����ֲ������Ĭ������PWMƵ��16kHz��Ӧ��
     */
    tim4_pwm_init(2111 - 1, 84 - 1); //Ħ���ֳ�ʼ��
	
#if (REVERSE_LIGHT_COUPLING == 1)
    TIM4->CCR1 = 2111-1000;
    TIM4->CCR2 = 2111-1000;
#else
    TIM4->CCR1 = 1000;
    TIM4->CCR2 = 1000;
#endif
}

void steering_gear_init(void)
{
    tim3_pwm_init(2000 - 1, 840 - 1); //���ָ� & ����̨P �����ʼ 50Hz
    
//  TIM3_PWM_6020_Init(2000 - 1, 840 - 1);
    TIM3->CCR1 = 0;                   //���ָǳ�ʼλ�� & ����̨P��ʼ��
    TIM3->CCR2 = 0;                   //���ݳ����Ƴ�ʼ0-200
}

/************************************************************TIM3 PWM��ʼ��**************************************************************/
/*
���ܣ����ֶ��/���ݳ�� PWM��ʼ��  TIM3  PA6��ch1�� PA7��ch2�� & ����̨p�� PA6
�β�1 arr���Զ���װ��ֵ
�β�2 psc����ʱ����Ƶ
*/
void tim3_pwm_init(u32 arr, u32 psc) //����=��arr+1��*��psc+1��/CLK 84m     Ƶ��=1/����      ռ�ձ� CCRx/Arr*100%
{
    //�ṹ������
    GPIO_InitTypeDef GPIO_InitStructure;           //�ṹ��_����-����
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //�ṹ��_ʱ������
    TIM_OCInitTypeDef TIM_OCInitStructure;         //PWM���

    //ʱ�ӿ���
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    //TIM3ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //ʹ��ʱ��
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); //GPIOB0����Ϊ��ʱ��3 �������
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); //GPIOB0����Ϊ��ʱ��3 �������

    //������������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;          //GPIOA6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       //����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //�ٶ�100MHz(����)
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //��ʼ��PA6

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;          //GPIOA7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       //����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //�ٶ�100MHz(����)
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //��ʼ��PA7

    //��ʼ����ʱ��
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  //��ʱ����Ƶ  psc
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr;                     //�Զ���װ��ֵ  arr
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    //��ʼ��TIM3 Channel12 PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);          //����Tָ���Ĳ�����ʼ������TIM3 OC3
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);          //����Tָ���Ĳ�����ʼ������TIM3 OC3
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM3, ENABLE); //ARPEʹ��

    //������ʱ��3
    TIM_Cmd(TIM3, ENABLE); //ʹ�ܶ�ʱ��3
}

void tim3_pwm_6020_init(u32 arr, u32 psc) //����=��arr+1��*��psc+1��/CLK 84m     Ƶ��=1/����      ռ�ձ� CCRx/Arr*100%
{
    //�ṹ������
    GPIO_InitTypeDef GPIO_InitStructure;           //�ṹ��_����-����
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //�ṹ��_ʱ������
    TIM_OCInitTypeDef TIM_OCInitStructure;         //PWM���

    //ʱ�ӿ���
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    //TIM3ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);   //ʹ��ʱ��
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); //GPIOB0����Ϊ��ʱ��3 �������

    //������������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       //����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //�ٶ�100MHz(����)
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;     //����
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //��ʼ����ʱ��
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  //��ʱ����Ƶ  psc
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr;                     //�Զ���װ��ֵ  arr
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    //��ʼ��TIM3 Channel12 PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);          //����Tָ���Ĳ�����ʼ������TIM3 OC3
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);          //����Tָ���Ĳ�����ʼ������TIM3 OC3
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM3, ENABLE); //ARPEʹ��

    //������ʱ��3
    TIM_Cmd(TIM3, ENABLE); //ʹ�ܶ�ʱ��3
}

/************************************************************TIM4 PWM��ʼ��**************************************************************/

/*
���ܣ�Ħ���ֳ�ʼ����PD12 PD13��
�β�1 arr���Զ���װ��ֵ
�β�2 psc����ʱ����Ƶ
*/
void tim4_pwm_init(u32 arr, u32 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //TIM4ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //ʹ��GPIODʱ��

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); //GPIOD12����Ϊ��ʱ��4
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); //GPIOD13����Ϊ��ʱ��4

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13; //GPIOD12 D13
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //����
    GPIO_Init(GPIOD, &GPIO_InitStructure);                   //��ʼ��GPIOH

    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  //��ʱ����Ƶ  84000/8400=10KHZ ��100us
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr;                     //�Զ���װ��ֵ            t=0.1ms*211=21.1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //��ʼ����ʱ��12

    //��ʼ��TIM4  PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);                      //����Tָ���Ĳ�����ʼ������TIM4 OC1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);                      //����Tָ���Ĳ�����ʼ������TIM4 OC2

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM4, ENABLE); //ARPEʹ��

    TIM_Cmd(TIM4, ENABLE); //ʹ��TIM4
}

/************************************************************TIM5 PWM��ʼ��**************************************************************/
/*
���ܣ� ����̨���pwm��ʼ��
�β�1 arr���Զ���װ��ֵ
�β�2 psc����ʱ����Ƶ
*/
void tim5_pwm_init(u32 arr, u32 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  //TIM5ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); //ʹ��GPIOHʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE); //ʹ��GPIOHʱ��

    GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5); //GPIOH10����Ϊ��ʱ��5
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5); //GPIOH11����Ϊ��ʱ��5
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource12, GPIO_AF_TIM5); //GPIOH12����Ϊ��ʱ��5
    GPIO_PinAFConfig(GPIOI, GPIO_PinSource0, GPIO_AF_TIM5);  //GPIOH9����Ϊ��ʱ��5

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_0; //GPIOH6.H9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                        //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                                  //�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                        //����
    GPIO_Init(GPIOH, &GPIO_InitStructure);                                              //��ʼ��GPIOH
    GPIO_Init(GPIOI, &GPIO_InitStructure);                                              //��ʼ��GPIOI

    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  //��ʱ����Ƶ  84000/8400=10KHZ ��100us
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr;                     //�Զ���װ��ֵ            t=0.1ms*211=21.1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //��ʼ����ʱ��12

    //��ʼ��TIM5  PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);                      //����Tָ���Ĳ�����ʼ������TIM5 OC1
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);                      //����Tָ���Ĳ�����ʼ������TIM5 OC2
    TIM_OC3Init(TIM5, &TIM_OCInitStructure);                      //����Tָ���Ĳ�����ʼ������TIM5 OC3
    TIM_OC4Init(TIM5, &TIM_OCInitStructure);                      //����Tָ���Ĳ�����ʼ������TIM5 OC4

    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); //ʹ��TIM5��CCR1�ϵ�Ԥװ�ؼĴ���
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable); //ʹ��TIM5��CCR2�ϵ�Ԥװ�ؼĴ���
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable); //ʹ��TIM5��CCR3�ϵ�Ԥװ�ؼĴ���
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); //ʹ��TIM5��CCR4�ϵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM5, ENABLE); //ARPEʹ��

    TIM_Cmd(TIM5, ENABLE); //ʹ��TIM12
}

/************************************************************TIM12 PWM��ʼ��**************************************************************/
/*
���ܣ�
�β�1 arr���Զ���װ��ֵ
�β�2 psc����ʱ����Ƶ
*/
void tim12_pwm_init(u32 arr, u32 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE); //TIM12ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); //ʹ��GPIOHʱ��

    GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_TIM12); //GPIOH6����Ϊ��ʱ��12
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource9, GPIO_AF_TIM12); //GPIOH9����Ϊ��ʱ��12

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9; //GPIOH6.H9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     //�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //����
    GPIO_Init(GPIOH, &GPIO_InitStructure);                 //��ʼ��GPIOH6, H9

    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  //��ʱ����Ƶ  84000/8400=10KHZ ��100us
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period = arr;                     //�Զ���װ��ֵ            t=0.1ms*211=21.1ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure); //��ʼ����ʱ��12

    //��ʼ��TIM12 Channel1 PWMģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);                     //����Tָ���Ĳ�����ʼ������TIM12 OC1
    TIM_OC2Init(TIM12, &TIM_OCInitStructure);                     //����Tָ���Ĳ�����ʼ������TIM12 OC2

    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable); //ʹ��TIM12��CCR1�ϵ�Ԥװ�ؼĴ���
    TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable); //ʹ��TIM12��CCR2�ϵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM12, ENABLE); //ARPEʹ��

    TIM_Cmd(TIM12, ENABLE); //ʹ��TIM12
}
