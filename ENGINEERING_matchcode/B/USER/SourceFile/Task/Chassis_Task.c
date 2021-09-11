#include "Chassis_Task.h"
#include "BoardCommuni.h"
#include "FreeRTOS.h"
#include "task.h"

#if 0
    static u8 Chassis_Wheel_Follow 		= 1;
    static u8 Chassis_Wheel_Indepen 	= 2;
    static u8 Chassis_Wheel_Wiggle		= 3;
    static u8 Chassis_Wheel_Off 			= 4;
    static u8 Chassis_Key							= 5;
    static u8 Last_ChassisWheelState	= 255;
#endif

extern Chassis_t Chassis;
Chassis_t Chassis;

/*Chassis_Task初始化*/
static void Chassis_Init(void)
{
    /*函数映射*/
    Chassis.Fsm_Init = Chassis_FSM_Init;
    Chassis.Wheel_Init = Wheel_Motor_Init;
    Chassis.Indepen = Chassis_Indepen_Drive;
    Chassis.Wiggle_Run = Chassis_Wiggle_Drive;
    Chassis.Rotation = Chassis_Rotation_Drive;
    Chassis.Poweroff = Chassis_Poweroff_Drive;
    Chassis.Auto_run = Chassis_Auto_Drive;
    Chassis.Rescue = Chassis_Rescue;
    Chassis.Barrier = Chassis_Barrier;
    /*新增功能*/
    Chassis.Straight_Drive = Chassis_Straight_Drive;
    Chassis.BoardCommuni_Update = BoardCommuni_DataUpdate;

    /*数据初始化*/
    Chassis.RC = Return_RemoteDeal_Point(); //Chassis的获取Remote数据指针
    Chassis.Wheel_Init(&Chassis.C);         //Chassis的轮子初始化
    Chassis.Fsm = Return_Chassis_FSM();     //Chassis获取状态机数据指针
    Chassis.Fsm_Init();                     //底盘状态机初始化

    BoardCommuni_Init();
}

void Chassis_Task(void *pvParameters)
{
    vTaskDelay(500);
    Chassis_Init();

    while (1)
    {
        FSM_Deal(Chassis.Fsm, Chassis.RC->RC_ctrl->s1, Chassis.RC->RC_ctrl->s2);
        vTaskDelay(2);
    }
}
