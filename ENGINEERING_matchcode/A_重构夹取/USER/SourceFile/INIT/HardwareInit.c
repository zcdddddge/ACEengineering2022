#include "HardwareInit.h"
#include "CAN.h"
#include "USART.h"
#include "TIM.h"
#include "GPIO.h"
#include "IWDG.h"

/*can初始化配置参数宏定义*/
#define can_configure_parameter_90M		CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal
#define can_configure_parameter_84M		CAN_SJW_1tq, CAN_BS2_5tq, CAN_BS1_8tq, 3, CAN_Mode_Normal

/*硬件初始化结构体*/
static Hardware_t Hardware;


/*硬件初始化映射*/
static void HardwareMapping(void)
{
	Hardware.SENSOR_Init= SENSOR_Init;
	Hardware.LED_Init  	= Led_Init;
	Hardware.TIM3_Init 	= TIM3_Init;
	Hardware.CAN1_Init 	= CAN1_Init_GPIOD_IO0_IO1;
	Hardware.CAN2_Init 	= CAN2_Init_GPIOB_IO5_IO6;
	Hardware.RC_Init	 	= USART1_RC_INIT_PA10_RX;
	Hardware.VL53L0			= USART6_VL53L0_INIT_PC7_RX_PC6_TX;
//	Hardware.Iwdg				= IWDG_Init;
}


/*硬件初始化*/
void HardwareInit(void)
{
	HardwareMapping();
	Hardware.SENSOR_Init();
	Hardware.LED_Init();
	Hardware.RC_Init();
	Hardware.VL53L0();
	Hardware.CAN1_Init(can_configure_parameter_84M);
	Hardware.CAN2_Init(can_configure_parameter_84M);
	//Hardware.TIM3_Init(84,1000);
	//Hardware.Iwdg(3,1000);
}

