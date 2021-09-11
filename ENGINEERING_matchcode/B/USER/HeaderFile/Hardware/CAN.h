#ifndef __CAN_H_
#define __CAN_H_
#include "stm32f4xx.h"

/*CAN1��ʼ�����ã���ӦIOΪPA11��PA12���ж����ȼ�Ϊ1*/
void CAN1_Init_GPIOA_IO11_IO12(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode);

/*CAN1��ʼ�����ã���ӦIOΪPD0��PD1���ж����ȼ�Ϊ1*/
void CAN1_Init_GPIOD_IO0_IO1(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode);

/*CAN2��ʼ�����ã���ӦIOΪPB12��PB13���ж����ȼ�Ϊ2*/
void CAN2_Init_GPIOB_IO12_IO13(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode);

/*CAN2��ʼ�����ã���ӦIOΪPB5��PB6���ж����ȼ�Ϊ2*/
void CAN2_Init_GPIOB_IO5_IO6(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode);

#endif
