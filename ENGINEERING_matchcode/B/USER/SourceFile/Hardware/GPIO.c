#include "GPIO.h"



/*************************************************************************************************
*����:	LED_Init
*����:	LED��ʼ������
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
#define GPIO_PIN GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4
void LED_Init(void)
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
			
	GPIO_SetBits(GPIOE,GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);	
	GPIO_ResetBits(GPIOE,GPIO_Pin_3 | GPIO_Pin_4);
	GPIO_ResetBits(GPIOB,GPIO_Pin_7);
}


/*************************************************************************************************
*����:	SENSOR_Init
*����:	��������ʼ������
*�β�: 	��
*����:	��
*˵��:	PA-1	9
*************************************************************************************************/
void SENSOR_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;								
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;								
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/*************************************************************************************************
*����:	SWITCH_Init
*����:	���ƿ���IO��ʼ��
*�β�: 	��
*����:	��
*˵��:	PC-3
*************************************************************************************************/
void SWITCH_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;								
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;								
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;						
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;								
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}



/*************************************************************************************************
*����:	LASER_Init
*����:	�����ʼ��
*�β�: 	��
*����:	��
*˵��:	PA12
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

