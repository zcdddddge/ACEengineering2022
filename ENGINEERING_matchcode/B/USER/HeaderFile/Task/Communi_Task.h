#ifndef	__COMMUNI_TASK_H_
#define __COMMUNI_TASK_H_
#include "BoardCommuni.h"
#include "RemoteDeal.h"

/*ͨ������ṹ��*/
typedef __packed struct
{
	/*���ݱ���*/
	REMOTE_t *Remote_C;

}Communi_t;

void Communi_Task(void *pvParameters);

#endif
