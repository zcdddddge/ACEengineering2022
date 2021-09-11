#ifndef __TIMER_SEND_TASK_H
#define __TIMER_SEND_TASK_H
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

void timer_send_create(void);
void timer_receive_create(void);
#endif
