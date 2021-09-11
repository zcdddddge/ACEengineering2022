#ifndef __CAN_H_
#define __CAN_H_
#include "stm32f4xx.h"

/*CAN1初始化配置，对应IO为PA11，PA12，中断优先级为1*/
void CAN1_Init_GPIOA_IO11_IO12(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode);

/*CAN1初始化配置，对应IO为PD0，PD1，中断优先级为1*/
void CAN1_Init_GPIOD_IO0_IO1(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode);

/*CAN2初始化配置，对应IO为PB12，PB13，中断优先级为2*/
void CAN2_Init_GPIOB_IO12_IO13(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode);

/*CAN2初始化配置，对应IO为PB5，PB6，中断优先级为2*/
void CAN2_Init_GPIOB_IO5_IO6(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode);

#endif
