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

/*Chassis_Task��ʼ��*/
static void Chassis_Init(void)
{
    /*����ӳ��*/
    Chassis.Fsm_Init = Chassis_FSM_Init;
    Chassis.Wheel_Init = Wheel_Motor_Init;
    Chassis.Indepen = Chassis_Indepen_Drive;
    Chassis.Wiggle_Run = Chassis_Wiggle_Drive;
    Chassis.Rotation = Chassis_Rotation_Drive;
    Chassis.Poweroff = Chassis_Poweroff_Drive;
    Chassis.Auto_run = Chassis_Auto_Drive;
    Chassis.Rescue = Chassis_Rescue;
    Chassis.Barrier = Chassis_Barrier;
    /*��������*/
    Chassis.Straight_Drive = Chassis_Straight_Drive;
    Chassis.BoardCommuni_Update = BoardCommuni_DataUpdate;

    /*���ݳ�ʼ��*/
    Chassis.RC = Return_RemoteDeal_Point(); //Chassis�Ļ�ȡRemote����ָ��
    Chassis.Wheel_Init(&Chassis.C);         //Chassis�����ӳ�ʼ��
    Chassis.Fsm = Return_Chassis_FSM();     //Chassis��ȡ״̬������ָ��
    Chassis.Fsm_Init();                     //����״̬����ʼ��

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
