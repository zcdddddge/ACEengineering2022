#ifndef __FSM_H_
#define __FSM_H_

#ifndef NULL 
	#define NULL 0 
#endif 

#define State_Line   3 
#define State_Column 3

typedef __packed  struct 
{
	void (*State_Prepare)(void) ;
	void (*State_Process)(void) ;
	void (*Behavior_Process)(void) ;
}State_t ; 

typedef __packed  struct 
{
	unsigned char (*State_Change)(State_t *L_Sta, State_t *C_Sta); 
	State_t *Last_State ;
	State_t *Current_State;
	State_t (*State_Table)[State_Column];
}FSM_t ;

void FSM_Deal(FSM_t *fsm, unsigned char s1 , unsigned char s2) ;

unsigned char StateChange(State_t *L_Sta,State_t *C_Sta);
#endif 

