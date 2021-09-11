#include "RobotFsm.h"
#include "RobotAction.h"

/*********************LIFT********************/
static void Lift_Prepare(Gr_t *Gr);
static void Lift_Action(Gr_t *Gr);
static void Lift_Exit(Gr_t *Gr);
/*********************ROLL********************/
static void Roll_Prepare(void);
static void Roll_Action(void);
static void Roll_Exit(void);
/*********************MINGLE********************/
static void Mingle_Prepare(void);
static void Mingle_Action(void);
static void Mingle_Exit(void);
/*********************STRENCH********************/
static void Strench_Prepare(void);
static void Strench_Action(void);
static void Strench_Exit(void);

/*状态机*/
Action_Fsm_t  Gr_Fsm   ;

/*状态机状态定义*/
Action_State_t LIFT;      //抬升
Action_State_t ROLL;      //翻转
Action_State_t MINGLE;    //夹子
Action_State_t STRETCH;   //伸缩
Action_State_t POWEROFF ;  //离线

/*夹取状态表定义*/
Action_State_t Grasp_Table[6] ;




void Grasp_Fsm_Init()
{
    Gr_Fsm.CurState  	= NULL ;
    Gr_Fsm.LastState 	= NULL;
    Gr_Fsm.State_modify = NULL ;
    Gr_Fsm.Grasp_Table  = Grasp_Table;

    LIFT.State_Prepare = Lift_Prepare;
    LIFT.State_Action  = NULL ;
    LIFT.State_Exit    = Lift_Exit ;

    ROLL.State_Prepare = Roll_Prepare;
    ROLL.State_Action  = NULL ;
    ROLL.State_Exit    = Roll_Exit;

    MINGLE.State_Prepare = Mingle_Prepare;
    MINGLE.State_Action  = NULL ;
    MINGLE.State_Exit    = Mingle_Exit;

    STRETCH.State_Prepare = Strench_Prepare;
    STRETCH.State_Action  = NULL ;
    STRETCH.State_Exit 	  = Strench_Exit;

    Gr_Fsm.Grasp_Table[0] = LIFT;

}

static void Lift_Prepare(Gr_t *Gr)
{
}
static void Lift_Action(Gr_t *Gr)
{
    //uplift(&Gr->GraspMotor[0], Gr->vL53L0, 10.0f, 1);
}
static void Lift_Exit(Gr_t *Gr)
{
}


static void Roll_Prepare(void)
{
}
static void Roll_Action(void)
{

}
static void Roll_Exit(void)
{

}
static void Mingle_Prepare(void)
{

}
static void Mingle_Action(void)
{

}
static void Mingle_Exit(void)
{

}

static void Strench_Prepare(void)
{

}
static void Strench_Action(void)
{

}
static void Strench_Exit(void)
{

}



unsigned char State_modify(Action_State_t *Last, Action_State_t *Curr)
{
    return (Last != Curr ? 1 : 0) ;
}