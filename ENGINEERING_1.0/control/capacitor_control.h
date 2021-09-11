#ifndef __CAPACITOR_CONTROL_H__
#define __CAPACITOR_CONTROL_H__
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "chassis_task.h"


void chassis_power_limit_control(chassis_control_t *chassis_pl_f);
void cap_timer_callback(TimerHandle_t xTimer);

#endif
