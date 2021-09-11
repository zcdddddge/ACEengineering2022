#ifndef	__COMMUNI_TASK_H_
#define __COMMUNI_TASK_H_
#include "BoardCommuni.h"
#include "RemoteDeal.h"

/*通信任务结构体*/
typedef __packed struct
{
	/*数据变量*/
	REMOTE_t *Remote_C;

}Communi_t;

void Communi_Task(void *pvParameters);

#endif
