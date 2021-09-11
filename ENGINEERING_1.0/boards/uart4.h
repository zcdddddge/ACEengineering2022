#ifndef __UART4_H__
#define __UART4_H__
#include "struct_typedef.h"

void uart4_init(uint8_t usart_NVIC);
void UART4_GYRO_INIT_PC11_RX_PC10_TX(void);
uint32_t uart4_dma_send(uint8_t *data, uint16_t len);
extern volatile unsigned char PYR_rx_buffer[64];						//µ×ÅÌÍÓÂİÒÇÊı¾İ»º³åÇø
#endif


