#include "Chassis_Task.h"
#include "FreeRTOS.h"
#include "task.h"

Chassis_t Chassis;

void Chassis_Task(void *pvParameters)
{
    Chassis.Get_RemoteDeal_Point = Return_RemoteDeal_Point;	//����ӳ��
    Chassis.Get_Chassis_Motor_Speed = Get_Chassis_Data;
    vTaskDelay(500);
    Chassis.Remote_C = Chassis.Get_RemoteDeal_Point();			//Chassis�Ļ�ȡRemote����ָ��
    Wheel_Motor_Init(&Chassis.C);														//Chassis�����ӳ�ʼ��
    Rescue_Motor_Init(&Chassis.C);													//Chassis�ľ�Ԯ��ʼ��

    while(1)
    {
        Chassis.Get_Chassis_Motor_Speed(&Chassis.C);
        Chassis_Follow_Drive(&Chassis.C, Chassis.Remote_C->RC_Y.Output, Chassis.Remote_C->RC_X.Output, Chassis.Remote_C->RC_Z.Output);
        vTaskDelay(2);
    }
}
