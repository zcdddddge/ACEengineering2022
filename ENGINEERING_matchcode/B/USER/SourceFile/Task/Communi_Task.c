#include "Communi_Task.h"
#include "FreeRTOS.h"
#include "task.h"

Communi_t Communi;

/*Communi_Task初始化*/
static void Communi_Init(void)
{


}

void Communi_Task(void *pvParameters)
{
    //vTaskDelay(500);
    Communi_Init();

    while(1)
    {
//		if(Communi.Remote_C->CtrlGraspState == Ctrl_Grasp_On)
//		{
//			/*板子情况未知则发送ready信号*/
//			if(Communi.BoardCommuni->BoardState == Board_Unknow)
//			{
//				Communi.send_raady();
//				Communi.BoardCommuni->BoardState = Wait_Board_Ready;
//			}
//			/*发送ready信号后，等待板子返回信号，接收到信号后立即发送任务*/
//			else if(Communi.BoardCommuni->BoardState == Wait_Board_Ready)
//			{
//				Communi.wait_ready();
//				if(Communi.BoardCommuni->BoardState == Board_Ready)
//				{
//					Communi.send_test_task();
//					Communi.BoardCommuni->BoardState = Wait_Board_Progress;
//				}
//			}
//			/*发送任务信号后，等待板子返回进度信号*/
//			else if(Communi.BoardCommuni->BoardState == Wait_Board_Progress)
//			{
//				Communi.wait_progress();
//			}
//			/*任务完成*/
//			else if(Communi.BoardCommuni->BoardState == Board_Finish)
//			{
//				Communi.BoardCommuni->BoardState = Board_Unknow;
//				Communi.Remote_C->CtrlGraspState = Ctrl_Grasp_Off;
//			}
//			else
//			{
//				Communi.Remote_C->CtrlGraspState = Ctrl_Grasp_Off;
//			}
//		}
//		else if(Communi.Remote_C->CtrlGraspState == Ctrl_Grasp_Off)
//		{
//			Communi.BoardCommuni->BoardState = Board_Unknow;
//		}
//		else
//		{
//
//		}
        vTaskDelay(2);
    }
}
