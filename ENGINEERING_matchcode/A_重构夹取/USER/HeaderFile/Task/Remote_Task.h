#ifndef __REMOTE_TASK_H_
#define __REMOTE_TASK_H_
#include "RemoteDeal.h"
#include "GraspMotor.h"
#include "BoardCommuni.h"
#include "RobotAction.h"
#include "fsm.h" 

/*宏定义要注意重构时一键替换不受影响*/
#define SupplyOpen           bulletSupply(&Grasp.Gr,&Grasp.Gr.GraspMotor[6],-1) 
#define SupplyClose          bulletSupply(&Grasp.Gr , &Grasp.Gr.GraspMotor[6],1) 

#define Supply(dire)         bulletSupply(&Grasp.Gr,&Grasp.Gr.GraspMotor[6],dire) 

void Remote_Task(void *pvParameters);


#endif
