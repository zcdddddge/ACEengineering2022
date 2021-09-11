#include "GPIO_DEAL.h"
#include "GPIO.h"

/*GPIOx*/
#define GPIO_1 	GPIOA
#define GPIO_2 	GPIOC
#define GPIO_3  GPIOB

/*GPIO_Pin_1*/
static const uint16_t YAW_Calibrate_PIN     = GPIO_Pin_1;
/*GPIO_Pin_12*/
static const uint16_t CAMERA_GRASP_PIN  		= GPIO_Pin_12;
/*GPIO_Pin_13*/
static const uint16_t CAMERA_RESCUE_PIN  		= GPIO_Pin_13;
/*��Ч��������*/
static const u8 FilteNum = 2;


/*YAW���д�����*/
u8 Return_YAW_Calibrate_Val(void)
{
	static int8_t I = 0;
	static u8 Val = 1;
	
	if(GPIO_ReadInputDataBit(GPIO_1,YAW_Calibrate_PIN) == 1)
	{
		I ++;
	}
	else if(GPIO_ReadInputDataBit(GPIO_1,YAW_Calibrate_PIN) == 0)
	{
		I --;
	}
	
	if(I >= FilteNum)
	{
		Val = 1;
		I = 0;
	}
	else if(I <= -FilteNum)
	{
		Val = 0;
		I = 0;
	}
	
	return Val;
}


/*��������ͷ*/
/*
	1 - ������ȡ����ͷ	2 - ������Ԯ����ͷ	3 - �ر���������ͷ
*/
void CAMERA_SWITCH(u8 choice)
{
	if(choice == 3)
	{
		GPIO_ResetBits(GPIO_3,CAMERA_GRASP_PIN);
		GPIO_ResetBits(GPIO_3,CAMERA_RESCUE_PIN);
	}
	else if(choice == 2)
	{
		GPIO_SetBits(GPIO_3,CAMERA_GRASP_PIN);
		GPIO_ResetBits(GPIO_3,CAMERA_RESCUE_PIN);
	}
	else if(choice == 1)
	{
		GPIO_SetBits(GPIO_3,CAMERA_RESCUE_PIN);
		GPIO_ResetBits(GPIO_3,CAMERA_GRASP_PIN);
	}
}
