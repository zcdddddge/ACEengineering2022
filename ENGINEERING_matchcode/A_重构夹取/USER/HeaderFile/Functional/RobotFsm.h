#ifndef __ROBOTFSM_H_
#define __ROBOTFSM_H_

#include "stdint.h" 
#include "RobotAction.h" 

#define NULL        0

typedef __packed struct 
{
    void (*State_Exit)(Gr_t *Gr); /*状态结束*/
    void (*State_Prepare)(Gr_t *Gr);
    void (*State_Action)(Gr_t *Gr); /*状态执行函数*/
}Action_State_t;

/*状态机: 当前状态,状态表,转态迁移,状态判断 根据电机的状态判断*/
typedef __packed  struct 
{
    unsigned char (*State_modify)(Action_State_t *Last, Action_State_t *Curr);  /*状态变更函数*/ 
    Action_State_t *CurState;
    Action_State_t *LastState;                    
    Action_State_t *Grasp_Table ;  /*夹取状态表 */ 
}Action_Fsm_t;



Action_Fsm_t  *Return_Grasp_Fsm(void); 

/*状态机初始化*/
void Grasp_Fsm_Init(void);

unsigned char State_modify(Action_State_t *Last, Action_State_t *Curr);

#endif 