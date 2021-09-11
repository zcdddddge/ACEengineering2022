#include "Remote_Task.h"
#include "Grasp_Task.h"

extern Grasp_t Grasp;


#define Lift_Up Grasp.Up_Init(&Grasp.Gr)
#define Lift_Down Grasp.Down_Init(&Grasp.Gr)//还没写这个函数记得补上



static u8 lock = 1 	;
//static uint8_t loop_lock = 1;
FSM_t Grasp_Fsm 	;
State_t AUTOGRASP	;
State_t KEYBOARD	;
State_t PICKGOLD	;
State_t OFFLINE		;
State_t GRASPRESET;    /*夹取复位*/
State_t REMOTEG		;
	
State_t GETGOLD		;

State_t Grasp_State_Table[State_Line][State_Column];


/******************自动夹取**************************/
//static void AutoGrasp_State(void) ;
//static void AutoGrasp_bhv(void);
//static void AtuoGrasp_Prepare(void) ;
/*****************键盘******************************/
static void KeyBoard_State(void) ;
static void KeyBoard_bhv(void);
static void KeyBoard_Prepare(void) ;
/***************捡*******************************/
static void Pick_State(void) ;
static void Pick_bhv(void);
static void Pick_Prepare(void) ;
/*****************离线*****************************/
static void Offline_State(void) ;
static void Offline_bhv(void);
static void Offline_Prepare(void) ;
/*****************遥控****************************/
static void Remote_State(void) ;
static void Remote_bhv(void);
static void Remote_Prepare(void) ;
/****************兑换金币*****************************/
static void Gold_State(void) ;
static void Gold_bhv(void);
static void Gold_Prepare(void) ;

void Chassis_FSM_Init(void)
{
    Grasp_Fsm.State_Table    = Grasp_State_Table;
    Grasp_Fsm.Last_State     = NULL;
    Grasp_Fsm.Current_State  = NULL;
    Grasp_Fsm.State_Change   = StateChange;

//    AUTOGRASP.Behavior_Process = NULL ;
//    AUTOGRASP.State_Prepare    = AutoGrasp_State ;
//    AUTOGRASP.State_Process    = AtuoGrasp_Prepare;

    KEYBOARD.Behavior_Process = NULL ;
    KEYBOARD.State_Prepare    = KeyBoard_State;// KeyBoard_State ;
    KEYBOARD.State_Process    = KeyBoard_Prepare;

    OFFLINE.Behavior_Process = NULL ;
    OFFLINE.State_Prepare    = Offline_State ;
    OFFLINE.State_Process    = Offline_Prepare;

    REMOTEG.Behavior_Process = NULL ;
    REMOTEG.State_Prepare    = Remote_State ;
    REMOTEG.State_Process    = Remote_Prepare;

    GETGOLD.Behavior_Process = NULL ;
    GETGOLD.State_Prepare    = Gold_State ;
    GETGOLD.State_Process    = Gold_Prepare;

    PICKGOLD.Behavior_Process = NULL ;
    PICKGOLD.State_Prepare    = Pick_State;
    PICKGOLD.State_Process    = Pick_Prepare ;

    /*底盘状态机初始化*/
//    Grasp_State_Table[0][0] = OFFLINE;      //s1=1 ,s2=1 离线
//    Grasp_State_Table[0][2] = OFFLINE;      //s1=1  s2=3 离线
//    Grasp_State_Table[0][1] = OFFLINE;      //s1=1  s2=2 离线
//    Grasp_State_Table[1][0] = OFFLINE;      //s1=2  s2=1 离线
//    Grasp_State_Table[1][1] = PICKGOLD;     //s1=2  s2=2 地上捡
//    Grasp_State_Table[1][2] = REMOTEG;      //s1=2  s2=3 遥控夹取
//    Grasp_State_Table[2][0] = KEYBOARD;     //s1=3 s2=1  键盘
//    Grasp_State_Table[2][1] = GETGOLD;      //s1=3 s2=2  兑换金币
//    Grasp_State_Table[2][2] = AUTOGRASP;    //s1=3 s2=3  自动夹取


		Grasp.Can2_RC->state.Grasp_Up = 0; //
		Grasp.Can2_RC->state.Translation = 0;
		Grasp.Can2_RC->state.Telescoping = 0; //
		Grasp.Can2_RC->state.Clap = 0;  //
		Grasp.Can2_RC->state.Flip = 0;


    Grasp_State_Table[0][0] = REMOTEG;      //s1=1 ,s2=1 离线
    Grasp_State_Table[0][2] = REMOTEG;      //s1=1  s2=3 离线
    Grasp_State_Table[0][1] = REMOTEG;      //s1=1  s2=2 离线
    Grasp_State_Table[1][0] = OFFLINE;      //s1=2  s2=1 离线
    Grasp_State_Table[1][1] = OFFLINE;     //s1=2  s2=2 地上捡
    Grasp_State_Table[1][2] = REMOTEG;      //s1=2  s2=3 遥控夹取
    Grasp_State_Table[2][0] = KEYBOARD;     //s1=3 s2=1  键盘
    Grasp_State_Table[2][1] = KEYBOARD;      //s1=3 s2=2  键盘
    Grasp_State_Table[2][2] = KEYBOARD;    //s1=3 s2=3  键盘



}

FSM_t *Return_Chassis_Fsm(void)
{
    return &Grasp_Fsm;
}

/*****************************Auto_Grasp*****************************************/
//static void AutoGrasp_State(void)
//{
//    Grasp_Fsm.Current_State->Behavior_Process = AutoGrasp_bhv;
//}
//static void AutoGrasp_bhv(void)
//{
//    if(lock == 1)
//    {
//        Grasp.Auto_Grasp(&Grasp.Gr, 1) ;
//        lock = 2 ; //上锁
//    }
//    else
//    {
//        Grasp.Auto_Grasp(&Grasp.Gr, 0) ;
//    }
//}
//static void AtuoGrasp_Prepare(void)
//{}

/******************************Offline************************************************************/
static void Offline_State(void)
{
    Grasp_Fsm.Current_State->Behavior_Process =  Offline_bhv;
}
static void Offline_bhv(void)
{
    lock = 1 ; //解锁
    Grasp.Grasp_Poweroff(&Grasp.Gr);
}
static void Offline_Prepare(void)
{

}

/*******************************Remote******************************************************/
static void Remote_State(void)
{
    Grasp_Fsm.Current_State->Behavior_Process = Remote_bhv;
}
static void Remote_bhv(void)
{
    Grasp.RC_Grasp(&Grasp.Gr, &Grasp.Can2_RC->Can_RC);
}
static void Remote_Prepare(void) {}


/**********************KeyBoard******************************************************/
static void KeyBoard_State(void)
{
    Grasp_Fsm.Current_State->Behavior_Process = KeyBoard_bhv;
}
static void KeyBoard_bhv(void)
{
    static uint8_t graspLock = 1 ;
	
    /*抬升*/
    if(Grasp.Can2_RC->state.Grasp_Up == 3)
    {
        DOWN;
    }
    else if(Grasp.Can2_RC->state.Grasp_Up == 1)
    {
        GRASP_LIFT ;
    }
    else if(Grasp.Can2_RC->state.Grasp_Up == 2)
    {
        CONVERSION_LIFT ;
    }
		
    /*平移*/
    if(Grasp.Can2_RC->state.Translation)
    {
       RALL_FORWARD;
    }
    else
    {
       RALL_BACK;
    }
		
    /*伸缩*/
    if(Grasp.Can2_RC->state.Telescoping)
    {
				SLIDE_FORWARD;
    }
    else
    {
				SLIDE_BACK;
    }
		
    /*翻转*/
    if(Grasp.Can2_RC->state.Flip)
    {
        TURN_OVER;
    }
    else
    {
        TIP_BACK;
    }
    /*夹子*/
    if(Grasp.Can2_RC->state.Clap)
    {
        CLAMPING;
    }
    else
    {
        PINE_CLIP ;
    }
		CAN1_201_To_204_SEND(Grasp.Gr.GraspMotor[0].SPID.Out,Grasp.Gr.GraspMotor[1].SPID.Out,Grasp.Gr.GraspMotor[2].SPID.Out,Grasp.Gr.GraspMotor[3].SPID.Out);//Grasp.Gr.GraspMotor[3].SPID.Out
//		CAN1_201_To_204_SEND(0,Grasp.Gr.GraspMotor[1].SPID.Out,0,Grasp.Gr.GraspMotor[3].SPID.Out);
//		CAN1_201_To_204_SEND(0,0,0,0);

		//Q-X-C-B-V-cB-cV-cX-B-V-cB-cV-R
		//E-V-B-cV-
}
static void KeyBoard_Prepare(void) {}



/*****************Gold*******************************/
static void Gold_State(void)
{
    Grasp_Fsm.Current_State->Behavior_Process = Gold_bhv;
}
static void Gold_bhv(void)
{
    Grasp.Change_Gold(&Grasp.Gr);
}
static void Gold_Prepare(void) {}



static void Pick_bhv(void)
{
    //锁检测
    if(Grasp.Gr.state[3] != 5)
    {
        //Grasp.Pick_Gold(&Grasp.Gr);
    }

}
static void Pick_State(void)
{
    Grasp_Fsm.Current_State->Behavior_Process = Pick_bhv;
}

static void Pick_Prepare(void)
{
    Grasp.Gr.state[3] = 0 ; //重新赋值状态、开锁
}
