#include "Communi_Task.h"
#include "FreeRTOS.h"
#include "task.h"

Communi_t Communi;

/*Communi_Task��ʼ��*/
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
//			/*�������δ֪����ready�ź�*/
//			if(Communi.BoardCommuni->BoardState == Board_Unknow)
//			{
//				Communi.send_raady();
//				Communi.BoardCommuni->BoardState = Wait_Board_Ready;
//			}
//			/*����ready�źź󣬵ȴ����ӷ����źţ����յ��źź�������������*/
//			else if(Communi.BoardCommuni->BoardState == Wait_Board_Ready)
//			{
//				Communi.wait_ready();
//				if(Communi.BoardCommuni->BoardState == Board_Ready)
//				{
//					Communi.send_test_task();
//					Communi.BoardCommuni->BoardState = Wait_Board_Progress;
//				}
//			}
//			/*���������źź󣬵ȴ����ӷ��ؽ����ź�*/
//			else if(Communi.BoardCommuni->BoardState == Wait_Board_Progress)
//			{
//				Communi.wait_progress();
//			}
//			/*�������*/
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
