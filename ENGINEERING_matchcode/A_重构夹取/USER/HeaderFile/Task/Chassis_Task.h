#ifndef __CHASSIS_TASK_H_
#define __CHASSIS_TASK_H_
#include "ChassisMotor.h"
#include "RemoteDeal.h"

typedef __packed struct
{
	C_t C;
	REMOTE_t *Remote_C;
	REMOTE_t* (*Get_RemoteDeal_Point)(void);
	void (*Get_Chassis_Motor_Speed)(C_t *C);
}Chassis_t;

void Chassis_Task(void *pvParameters);

#endif
