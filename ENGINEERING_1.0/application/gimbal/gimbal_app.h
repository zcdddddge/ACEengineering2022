#ifndef __GIMBAL_APP_H__
#define __GIMBAL_APP_H__
#include "struct_typedef.h"
#include "freertos.h"
#include "task.h"

TaskHandle_t *get_gimbal_task_handle(void);

extern void gimbal_app_init(void);

extern void can1_gimbal_setmsg(int16_t ESC_201, int16_t ESC_202);
extern void can2_gimbal_setmsg(u8 Initialize_flag, u8 Yaw_zero_flag, u8 Supply_flag, int16_t gimbal_dif_angel, uint8_t gimbal_beh, uint16_t pitch_angle);
extern void can2_yaw_setmsg(int16_t gimbal_output);
extern void can1_function_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);
extern void can1_flip_setmsg(int16_t ESC_205);
extern void can2_sensor_setmsg(u8 sensor_l, u8 sensor_r);
#endif // __GIMBAL_APP_H__
