#ifndef	__HARDWAREINIT_H_
#define __HARDWAREINIT_H_

/*硬件初始化表*/
typedef __packed struct
{
		void (*CAN1_Init)(unsigned char, unsigned char, unsigned char, unsigned short, unsigned char);
		void (*CAN2_Init)(unsigned char, unsigned char, unsigned char, unsigned short, unsigned char);
		void (*TIM3_Init)(unsigned short,unsigned short);
		void (*SENSOR_Init)(void);
		void (*LED_Init)(void);
		void (*RC_Init)(void);
		void (*VL53L0)(void);
		void (*Iwdg)(unsigned char,unsigned short);
}Hardware_t;


/*硬件初始化*/
void HardwareInit(void);
#endif

