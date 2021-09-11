#ifndef __USART1_H__
#define __USART1_H__
#include "struct_typedef.h"
#include "rc.h"


void usart1_init(uint8_t usart_NVIC, uint16_t usart1_mode);
extern void (*rc_callback)(volatile const uint8_t *);
uint32_t usart1_dma_send(uint8_t *data, uint16_t len);

#endif


