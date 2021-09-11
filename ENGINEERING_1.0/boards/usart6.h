#ifndef __USART6_H__
#define __USART6_H__
#include "struct_typedef.h"


void usart6_init(uint8_t usart_NVIC);
extern volatile unsigned char vL53L0_rx_buffer[128];				//VL53L0Êı¾İ»º³åÇø
extern uint8_t (*usart6_callback)(uint8_t *, uint16_t);
uint32_t usart6_dma_send(uint8_t *data, uint16_t len);

#endif


