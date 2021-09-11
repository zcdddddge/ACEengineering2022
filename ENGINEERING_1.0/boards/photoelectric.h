#ifndef __PHOTOELECTRIC_H__
#define __PHOTOELECTRIC_H__
#include "struct_typedef.h"
#include "gimbal_task.h"


extern void photoelectric_gpio_init(void);
extern void yaw_zero_gpio_init(void);     //���У׼������
extern void yaw_two_zero_gpio_init(void);

extern uint8_t yaw_zero_value(void);
extern uint8_t sec_yaw_zero_value(void);   //���y���Ƿ�λ   ����ֵ����0�����򷵻�1
extern void correct_yaw_zero(gimbal_control_t *fir_gimbal_correct);
//void Correct_Sec_Yaw_Zero(gimbal_second_control_t *sec_gimbal_correct);
#endif
