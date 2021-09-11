#ifndef __USART_H_
#define __USART_H_
#include "stm32f4xx.h"

/*�ⲿ����*/
extern volatile unsigned char dbus_rx_buffer[25];						//ң�����ݻ�����
extern volatile unsigned char gyro_rx_buffer[25];						//��̨���������ݻ�����
extern volatile unsigned char vision_rx_buffer[32];					//�Ӿ����ݻ�����
extern volatile unsigned char referee_rx_buffer[128];				//����ϵͳ���ݻ�����
extern volatile unsigned char PYR_rx_buffer[64];						//�������������ݻ�����

/*USART1��ʼ����������RC��rx��ӦIOΪPB7���ж����ȼ�Ϊ0*/
void USART1_RC_INIT_PB7_RX(void);

/*USART1��ʼ����������RC��rx��ӦIOΪPA10���ж����ȼ�Ϊ0*/
void USART1_RC_INIT_PA10_RX(void);

/*USART6��ʼ����������RC��rx��ӦIOΪPC7���ж����ȼ�Ϊ0*/
void USART6_RC_INIT_PC7_RX(void);

/*USART6��ʼ����������REFEREE��rx��ӦIOΪPG9��tx��ӦIOΪPG14���ж����ȼ�Ϊ3*/
void USART6_REFEREE_INIT_PG9_RX_PG14_TX(void);

/*USART3��ʼ����������VISION��rx��ӦIOΪPD9��tx��ӦIOΪPD8���ж����ȼ�Ϊ4*/
void USART3_VISION_INIT_PD9_RX_PD8_TX(void);

/*USART2��ʼ����������GYRO��rx��ӦIOΪPD6��tx��ӦIOΪPD5���ж����ȼ�Ϊ5*/
void USART2_GYRO_INIT_PD6_RX_PD5_TX(void);

/*UART4��ʼ���������ڵ���GYRO��rx��ӦIOΪPC11��tx��ӦIOΪPC10���ж����ȼ�Ϊ6*/
void UART4_GYRO_INIT_PC11_RX_PC10_TX(void);

/*ʧ�ܴ���*/
void USART_Disable(USART_TypeDef* USARTx);

#endif
