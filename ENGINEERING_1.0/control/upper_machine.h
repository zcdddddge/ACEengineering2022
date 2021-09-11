#ifndef __UPPER_MACHINE_H__
#define __UPPER_MACHINE_H__
#include "struct_typedef.h"
#include "pid.h"

void upper_machine_communication(void);
void upper_machine_usart2_callback(uint8_t *usart2_data, uint16_t Len);
void pid_parameter_receive(pid_parameter_t *pid_speed, pid_parameter_t *pid_position);
void vofa_test(void);
#endif















