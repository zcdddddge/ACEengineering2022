#ifndef __CHASSIA_FSM_H_
#define __CHASSIA_FSM_H_
#include "fsm.h"

/*底盘状态机初始化*/
void Chassis_FSM_Init(void);

/*返回底盘状态机控制指针*/
FSM_t *Return_Chassis_FSM(void);
#endif
