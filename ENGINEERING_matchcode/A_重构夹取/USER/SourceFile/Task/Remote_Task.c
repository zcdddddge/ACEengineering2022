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
			
			
			
			
///* ̧������ */
//			uplift(&Grasp.Gr.GraspMotor[0], Grasp.Gr.vL53L0, 12.8f, 1);
//			

///* ǰ���� */
//			if(Grasp.Gr.GraspMotor[0].state == Finish)
//			{
//						Grasp.Gr.GraspMotor[2].state = DisFinish;
//			}
//			if(Grasp.Gr.GraspMotor[2].state == DisFinish)
//				{
//					Telescoping(&Grasp.Gr.GraspMotor[2], 1);//һ��Ҫ���̧��һ����ܿ�
//				}
///* ��ת��� */
//				if(Grasp.Gr.GraspMotor[2].state == Finish)
//				{
//					flip2(&Grasp.Gr.GraspMotor[4], 170, 10, 1);//һ��Ҫ��׼������ȷ���ܿ�
//				
//					Translation(&Grasp.Gr.GraspMotor[1],1);

//				}
//			if(Grasp.Gr.GraspMotor[1].state == Finish && Grasp.Gr.GraspMotor[4].state == Finish)
//			{
//					clip(&Grasp.Gr.GraspMotor[3], Cilp_Speed, 1);
//			}

//					pid_Cala(&Grasp.Gr);

		
/*debug��ʱ��㿪view,ѡ��register,����LR��ֵ�жϣ�ʹ�õ���psp��ջ��Ȼ���memory windows���鿴��ַΪ0x20002FF0���ڴ����ݣ���Ϊ���һ����ջ�����ݡ�
�Ҽ�ѡ��long��ʾview,disassembly window������ڣ��鿴������ļ����ڷ���ര�ڣ��Ҽ������show disassembly at address �������ַ���Ϳ����ҵ���Ӧ����ļ���λ�ã�
ͬʱ���Զ�λ��c�����ж�Ӧ��λ�á�
����һ�ַ�ʽ���ǣ�ͨ���������ĵ��ù�ϵ��ֱ�ӿ������������������
��� view call stack window ,ֱ�ӿ��Կ�������������֮ǰ�ĺ������ù�ϵ*/
        vTaskDelay(2);
    }
}

