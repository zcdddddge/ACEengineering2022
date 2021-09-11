#ifndef	__HARDWAREINIT_H_
#define __HARDWAREINIT_H_

/*硬件初始化表*/
typedef __packed struct
{
		void (*RC_Init)(void);
		void (*REFEREE_Init)(void);
		void (*VISION_Init)(void);
		void (*GYRO_Init)(void);
		void (*gyro_init)(void);
		void (*CAN1_Init)(unsigned char, unsigned char, unsigned char, unsigned short, unsigned char);
		void (*CAN2_Init)(unsigned char, unsigned char, unsigned char, unsigned short, unsigned char);
		void (*TIM3_Init)(unsigned short,unsigned short);
		void (*LED_Init)(void);
		void (*Electromagnet_Init)(void);
	  void (*Iwdg)(unsigned char,unsigned short);
}Hardware_t;


/*硬件初始化*/
void HardwareInit(void);
#endif

