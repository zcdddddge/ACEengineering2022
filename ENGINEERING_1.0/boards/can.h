#ifndef __CAN_H__
#define __CAN_H__
#include "struct_typedef.h"

uint8_t can1_mode_init(uint8_t sjw, uint8_t bs1, uint8_t bs2, uint8_t prescaler, uint8_t mode);
uint8_t can2_mode_init(uint8_t sjw, uint8_t bs1, uint8_t bs2, uint8_t prescaler, uint8_t mode);

#endif /* __CAN_H__ */














