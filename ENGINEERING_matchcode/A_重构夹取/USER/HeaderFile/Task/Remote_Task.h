#ifndef __REMOTE_TASK_H_
#define __REMOTE_TASK_H_
#include "RemoteDeal.h"
#include "GraspMotor.h"
#include "BoardCommuni.h"
#include "RobotAction.h"
#include "fsm.h" 

/*�궨��Ҫע���ع�ʱһ���滻����Ӱ��*/
#define SupplyOpen           bulletSupply(&Grasp.Gr,&Grasp.Gr.GraspMotor[6],-1) 
#define SupplyClose          bulletSupply(&Grasp.Gr , &Grasp.Gr.GraspMotor[6],1) 

#define Supply(dire)         bulletSupply(&Grasp.Gr,&Grasp.Gr.GraspMotor[6],dire) 

void Remote_Task(void *pvParameters);


#endif
