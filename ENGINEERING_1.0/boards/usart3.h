#ifndef __USART3_H__
#define __USART3_H__
#include "struct_typedef.h"



void usart3_init(uint8_t usart_NVIC);
uint32_t usart3_dma_send(uint8_t *data, uint16_t len);

#endif


