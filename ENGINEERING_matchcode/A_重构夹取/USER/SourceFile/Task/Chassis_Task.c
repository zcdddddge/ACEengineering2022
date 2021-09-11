#include "Chassis_Task.h"
#include "FreeRTOS.h"
#include "task.h"

Chassis_t Chassis;

void Chassis_Task(void *pvParameters)
{
    Chassis.Get_RemoteDeal_Point = Return_RemoteDeal_Point;	//函数映射
    Chassis.Get_Chassis_Motor_Speed = Get_Chassis_Data;
    vTaskDelay(500);
    Chassis.Remote_C = Chassis.Get_RemoteDeal_Point();			//Chassis的获取Remote数据指针
    Wheel_Motor_Init(&Chassis.C);														//Chassis的轮子初始化
    Rescue_Motor_Init(&Chassis.C);													//Chassis的救援初始化

    while(1)
    {
        Chassis.Get_Chassis_Motor_Speed(&Chassis.C);
        Chassis_Follow_Drive(&Chassis.C, Chassis.Remote_C->RC_Y.Output, Chassis.Remote_C->RC_X.Output, Chassis.Remote_C->RC_Z.Output);
        vTaskDelay(2);
    }
}
