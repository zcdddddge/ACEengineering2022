#include "fsm.h"

void FSM_Deal(FSM_t *fsm, unsigned char s1, unsigned char s2)
{
    if( s1 > State_Line || s2 > State_Column)
        return ;

    if(s1 == 0 && s2 == 0)
    {
        s1 = 1 ;
        s2 = 1;
    }

    fsm->Current_State = &fsm->State_Table[s1 - 1][s2 - 1] ;

    if(fsm->State_Change(fsm->Last_State, fsm->Current_State))
    {
        fsm->Current_State->State_Prepare() ; //进入新状态，状态更新
    }

    fsm->Last_State = fsm->Current_State;

    fsm->Current_State->State_Process() ;

    fsm->Current_State->Behavior_Process() ;
}


unsigned char StateChange(State_t *L_Sta, State_t *C_Sta)
{
    return (L_Sta != C_Sta ? 1 : 0 ) ;
}

