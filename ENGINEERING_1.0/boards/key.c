#include "key.h"
#include "delay.h"
#include "led.h"

#define KEY PDin(10)

void key_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //ʹ��GPIODʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;         //KEY��Ӧ����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       //��ͨ����ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100M
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
    GPIO_Init(GPIOD, &GPIO_InitStructure);             //��ʼ��GPIOD10
}

u8 key_scan(u8 mode)
{
    static u8 key_up = 1;

    if (mode)
        key_up = 1;

    if (key_up && KEY == 0)
    {
        key_up = 0;
        delay_ms(10);

        if (KEY == 0)
        {
            return 1;
        }
    }
    else if (KEY == 1)
        key_up = 1;

    return 0;
}

void sensor_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;								
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;								
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}














