#include "led.h"

//E2 E3 E4 E0 E1 B7
void led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //ʹ��GPIOEʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //ʹ��GPIOEʱ��

    //GPIOE��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //��ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOB, &GPIO_InitStructure);             //��ʼ��

    GPIO_SetBits(GPIOE, GPIO_Pin_2); //GPIOE10 ���øߣ�����
    GPIO_SetBits(GPIOE, GPIO_Pin_3); //GPIOE12 ���øߣ�����
    GPIO_SetBits(GPIOE, GPIO_Pin_4); //GPIOE12 ���øߣ�����
    GPIO_SetBits(GPIOE, GPIO_Pin_0); //GPIOE10 ���øߣ�����
    GPIO_SetBits(GPIOE, GPIO_Pin_1); //GPIOE12 ���øߣ�����
    GPIO_SetBits(GPIOB, GPIO_Pin_7); //GPIOE12 ���øߣ�����
}

/*********����ģ���ʼ��**********/
void laser_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*********�����*****************/
void laser_on(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_12); //GPIOG13  ���øߣ�������
}
/*********�����*****************/
void laser_off(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_12); //GPIOG13  ���õͣ������
}

//LED��ʾ����
void led_task(void)
{
    //6��LED��ˮ��
    static u32 led_time = 0;
    static u8 led_choose = 0;
    led_time++;

    if (led_time % 5 == 0)
    {
        led_choose++;

        if (led_choose == 4)
            LEDE2 = !LEDE2;

        if (led_choose == 5)
            LEDE3 = !LEDE3;

        if (led_choose == 6)
        {
            LEDE4 = !LEDE4;
            led_choose = 0;
        }

        if (led_choose == 1)
            LEDB7 = !LEDB7;

        if (led_choose == 2)
            LEDE0 = !LEDE0;

        if (led_choose == 3)
            LEDE1 = !LEDE1;
    }
}
