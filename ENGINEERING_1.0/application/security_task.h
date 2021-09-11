#ifndef __SECURITY_TASK_H__
#define __SECURITY_TASK_H__
#include "struct_typedef.h"


/*全局函数声明*/
void safecheck_task(void *pvparameters);
void outctl_task(void *pvparameters);


extern uint32_t (*rc_lost_time)(void);
void system_out_ctrl_protect(void);
#endif
