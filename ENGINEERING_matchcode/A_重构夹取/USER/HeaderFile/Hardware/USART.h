#ifndef __USART_H_
#define __USART_H_
#include "stm32f4xx.h"

/*外部变量*/
extern volatile unsigned char dbus_rx_buffer[25];						//遥控数据缓冲区
extern volatile unsigned char gyro_rx_buffer[25];						//陀螺仪数据缓冲区
extern volatile unsigned char vision_rx_buffer[32];					//视觉数据缓冲区
extern volatile unsigned char referee_rx_buffer[128];				//裁判系统数据缓冲区
extern volatile unsigned char vL53L0_rx_buffer[128];				//VL53L0数据缓冲区

/*USART1初始化，适用于RC，rx对应IO为PB7，中断优先级为0*/
void USART1_RC_INIT_PB7_RX(void);

/*USART1初始化，适用于RC，rx对应IO为PA10，中断优先级为0*/
void USART1_RC_INIT_PA10_RX(void);

/*USART6初始化，适用于VL53L0测距，rx对应IO为PC7，tx对应PC6，中断优先级为,3*/
void USART6_VL53L0_INIT_PC7_RX_PC6_TX(void);

/*USART6初始化，适用于REFEREE，rx对应IO为PG9，tx对应IO为PG14，中断优先级为3*/
void USART6_REFEREE_INIT_PG9_RX_PG14_TX(void);

/*USART3初始化，适用于VISION，rx对应IO为PD9，tx对应IO为PD8，中断优先级为4*/
void USART3_VISION_INIT_PD9_RX_PD8_TX(void);

/*USART2初始化，适用于GYRO，rx对应IO为PD6，tx对应IO为PD5，中断优先级为5*/
void USART2_GYRO_INIT_PD6_RX_PD5_TX(void);

/*失能串口*/
void USART_Disable(USART_TypeDef* USARTx);

#endif
