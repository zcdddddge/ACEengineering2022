#include "stm32f4xx.h"
#include "HardwareInit.h"
#include "TaskInit.h"
#include "CAN1_ISR.h"

/*��ʼ������*/
static TaskHandle_t Start_Handler;

/*��ʼ������*/
static void Start_Task(void *pvParameters)
{
		taskENTER_CRITICAL();                                   //�����ٽ���
		
		TaskInit();
	
		vTaskDelete(Start_Handler);                     			//ɾ����ʼ����
    taskEXIT_CRITICAL();                                 	//�˳��ٽ���
}


int main()

{
	HardwareInit();
	xTaskCreate((TaskFunction_t )Start_Task,            	//������
			(const char*    )"Start_Task",                    //��������
			(uint16_t       )256,                  						//�����ջ��С
			(void*          )NULL,                            //���ݸ��������Ĳ���
			(UBaseType_t    )1,                 							//�������ȼ�
			(TaskHandle_t*  )&Start_Handler);            			//������  
			vTaskStartScheduler();                            //�����������
}


