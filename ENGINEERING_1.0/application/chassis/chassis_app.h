#ifndef __CHASSIS_APP_H__
#define __CHASSIS_APP_H__
#include "chassis_task.h"
#include "struct_typedef.h"
#include "rc.h"
#include "freertos.h"
#include "task.h"

TaskHandle_t *get_chassis_task_handle(void);

extern void chassis_app_init(void);

extern void can1_chassis_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);
extern void can1_chassis_gimbal_fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207);
extern void can1_cap_setmsg(int16_t Chassis_power);

extern void can1_rescue_setmsg(int16_t ESC_205, int16_t ESC_206);
extern void can1_spitch_setmsg(int16_t ESC_208);
extern void can1_syaw_setmsg(int16_t ESC_207);
	
extern void can2_chassis_rc_setmsg(const rc_ctrl_t *can2_RC_send);
extern void can2_chassis_mk_setmsg(const rc_ctrl_t *can2_MK_send);
extern void can2_4002_send(void);
extern void can2_gimbal_setmsg_2(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208);
extern void can2_chassis_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);

#endif

