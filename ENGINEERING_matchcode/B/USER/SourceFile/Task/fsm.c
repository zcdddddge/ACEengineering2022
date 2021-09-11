#include "fsm.h"

/*状态机处理*/
void FSM_Deal(FSM_t *fsm, unsigned char s1, unsigned char s2)
{
    /*误操作判断*/
    if(s1 <= 0 || s1 > State_Line || s2 <= 0 || s2 > State_Column)
    {
        return;
    }

    /*状态指向*/
    fsm->Current_State = &fsm->State_Table[s1 - 1][s2 - 1];

    /*状态变化*/
    if( fsm->State_Change(fsm->Last_State, fsm->Current_State) == 1)
    {
        fsm->Current_State->State_Prepare();
    }

    /*保留上次状态*/
    fsm->Last_State = fsm->Current_State;

    /*执行状态*/
    fsm->Current_State->State_Process();

    /*执行实际行为*/
    fsm->Current_State->Behavior_Process();
}


/*状态变更函数*/
unsigned char StateChange(State_t *L_Sta, State_t *C_Sta)
{
    return ( L_Sta != C_Sta ? (1) : (0) );
}
