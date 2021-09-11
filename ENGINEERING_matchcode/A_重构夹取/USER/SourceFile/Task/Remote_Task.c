#include "Remote_Task.h"
#include "FreeRTOS.h"
#include "Grasp_Task.h"
#include "task.h"

extern Grasp_t Grasp;

void Remote_Task(void *pvParameters)
{
    vTaskDelay(500);
	  Grasp.Can2_RC = Return_Board_Communi();
	
	

    while(1)
    {
			
			
			
			
///* 抬升测试 */
//			uplift(&Grasp.Gr.GraspMotor[0], Grasp.Gr.vL53L0, 12.8f, 1);
//			

///* 前伸电机 */
//			if(Grasp.Gr.GraspMotor[0].state == Finish)
//			{
//						Grasp.Gr.GraspMotor[2].state = DisFinish;
//			}
//			if(Grasp.Gr.GraspMotor[2].state == DisFinish)
//				{
//					Telescoping(&Grasp.Gr.GraspMotor[2], 1);//一定要配合抬升一起才能开
//				}
///* 翻转电机 */
//				if(Grasp.Gr.GraspMotor[2].state == Finish)
//				{
//					flip2(&Grasp.Gr.GraspMotor[4], 170, 10, 1);//一定要看准数据正确才能开
//				
//					Translation(&Grasp.Gr.GraspMotor[1],1);

//				}
//			if(Grasp.Gr.GraspMotor[1].state == Finish && Grasp.Gr.GraspMotor[4].state == Finish)
//			{
//					clip(&Grasp.Gr.GraspMotor[3], Cilp_Speed, 1);
//			}

//					pid_Cala(&Grasp.Gr);

		
/*debug的时候点开view,选择register,根据LR的值判断，使用的是psp堆栈，然后打开memory windows，查看地址为0x20002FF0的内存数据，即为最后一次入栈的内容。
右键选择，long显示view,disassembly window这个窗口，查看反汇编文件，在反汇编窗口，右键，点击show disassembly at address ，输入地址，就可以找到对应汇编文件的位置，
同时可以定位到c语言中对应的位置。
还有一种方式就是，通过看函数的调用关系，直接看到程序是死在那里的
点击 view call stack window ,直接可以看到，程序死机之前的函数调用关系*/
        vTaskDelay(2);
    }
}

