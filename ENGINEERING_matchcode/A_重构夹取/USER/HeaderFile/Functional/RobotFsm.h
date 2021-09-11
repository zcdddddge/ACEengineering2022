#ifndef __ROBOTFSM_H_
#define __ROBOTFSM_H_

#include "stdint.h" 
#include "RobotAction.h" 

#define NULL        0

typedef __packed struct 
{
    void (*State_Exit)(Gr_t *Gr); /*״̬����*/
    void (*State_Prepare)(Gr_t *Gr);
    void (*State_Action)(Gr_t *Gr); /*״ִ̬�к���*/
}Action_State_t;

/*״̬��: ��ǰ״̬,״̬��,ת̬Ǩ��,״̬�ж� ���ݵ����״̬�ж�*/
typedef __packed  struct 
{
    unsigned char (*State_modify)(Action_State_t *Last, Action_State_t *Curr);  /*״̬�������*/ 
    Action_State_t *CurState;
    Action_State_t *LastState;                    
    Action_State_t *Grasp_Table ;  /*��ȡ״̬�� */ 
}Action_Fsm_t;



Action_Fsm_t  *Return_Grasp_Fsm(void); 

/*״̬����ʼ��*/
void Grasp_Fsm_Init(void);

unsigned char State_modify(Action_State_t *Last, Action_State_t *Curr);

#endif 