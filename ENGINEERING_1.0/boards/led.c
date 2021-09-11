#include "led.h"

//E2 E3 E4 E0 E1 B7
void led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //使能GPIOE时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能GPIOE时钟

    //GPIOE初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);             //初始化

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);             //初始化

    GPIO_SetBits(GPIOE, GPIO_Pin_2); //GPIOE10 设置高，灯灭
    GPIO_SetBits(GPIOE, GPIO_Pin_3); //GPIOE12 设置高，灯灭
    GPIO_SetBits(GPIOE, GPIO_Pin_4); //GPIOE12 设置高，灯灭
    GPIO_SetBits(GPIOE, GPIO_Pin_0); //GPIOE10 设置高，灯灭
    GPIO_SetBits(GPIOE, GPIO_Pin_1); //GPIOE12 设置高，灯灭
    GPIO_SetBits(GPIOB, GPIO_Pin_7); //GPIOE12 设置高，灯灭
}

/*********激光模块初始化**********/
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
/*********激光打开*****************/
void laser_on(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_12); //GPIOG13  设置高，激光亮
}
/*********激光打开*****************/
void laser_off(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_12); //GPIOG13  设置低，激光关
}

//LED显示任务
void led_task(void)
{
    //6个LED流水灯
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
