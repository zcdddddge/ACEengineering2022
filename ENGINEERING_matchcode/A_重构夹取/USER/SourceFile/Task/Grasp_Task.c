#include "Grasp_Task.h"
#include "FreeRTOS.h"
#include "task.h"

Grasp_t Grasp;


/*Grasp_Task初始化*/
static void Grasp_Init(void)
{
    /*函数映射*/
    Grasp.Motor_Init		  		= Grasp_Motor_Init;
    Grasp.Auto_Grasp          = auto_grasp;
    Grasp.Grasp_Reset         = Reset_Grasp;
		Grasp.Flipmotor_Reset   	= Reset;
    Grasp.Grasp_Poweroff      = Poweroff_Ctrl;
    Grasp.Grasp_Fsm_Init      = Chassis_FSM_Init;
		Grasp.RC_Grasp            = RC_Ctrl;
//    Grasp.Bullet_Supply       = bullet_Supply;

//    Grasp.Send_Crtl           = Send_Crtl_To_Board;
//    Grasp.Change_Gold      		= change_Gold;
    //Grasp.Pick_Gold           = pick_Gold;


//    Grasp.Up_Init           = Grasp_init;
//		Grasp.Down_Init					= Grasp_reset;
//    Grasp.Grasp_The_First   = Grasp_First;
//    Grasp.Grasp_The_Second  = Grasp_Second;
//    Grasp.Grasp_The_Third   = Grasp_Third;

 //   Grasp.Allmotor_Reset    = All_reset;暂时没写 记得补上

    /*数据初始化*/
    Grasp.Motor_Init(&Grasp.Gr);	//夹取电机初始化
		Grasp.Fsm                 = Return_Chassis_Fsm();
		Grasp.Grasp_Fsm_Init();
}

void Grasp_Task(void *pvParameters)
{
    vTaskDelay(500);
    Grasp_Init();

    while(1)
    {
        FSM_Deal(Grasp.Fsm, Grasp.Can2_RC->Can_RC.s1, Grasp.Can2_RC->Can_RC.s2);

        vTaskDelay(2);
    }
}
