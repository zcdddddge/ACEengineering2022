#include "GPIO.h"



/*************************************************************************************************
*名称:	LED_Init
*功能:	LED初始化配置
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
#define GPIO_PIN GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4
void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;								
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;								
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN;								
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;								
	GPIO_Init(GPIOB, &GPIO_InitStructure);		
			
	GPIO_SetBits(GPIOE,GPIO_PIN);	
	GPIO_SetBits(GPIOB,GPIO_Pin_7);
}


/*************************************************************************************************
*名称:	SENSOR_Init
*功能:	传感器初始化配置
*形参: 	无
*返回:	无
*说明:	PA-6	7
*************************************************************************************************/
void SENSOR_Init(void)
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


/*************************************************************************************************
*名称:	SWITCH_Init
*功能:	控制开关IO初始化
*形参: 	无
*返回:	无
*说明:	PC-3	10	11
*************************************************************************************************/
void SWITCH_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;								
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;								
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}



/*************************************************************************************************
*名称:	LASER_Init
*功能:	激光初始化
*形参: 	无
*返回:	无
*说明:	PA12
*************************************************************************************************/
void LASER_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;								
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;								
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

