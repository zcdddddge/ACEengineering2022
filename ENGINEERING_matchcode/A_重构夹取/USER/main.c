#include "stm32f4xx.h"
#include "HardwareInit.h"
#include "TaskInit.h"
#include "CAN1_ISR.h"

/*开始任务句柄*/
static TaskHandle_t Start_Handler;

/*开始任务函数*/
static void Start_Task(void *pvParameters)
{
		taskENTER_CRITICAL();                                   //进入临界区
		
		TaskInit();
	
		vTaskDelete(Start_Handler);                     			//删除开始任务
    taskEXIT_CRITICAL();                                 	//退出临界区
}


int main()

{
	HardwareInit();
	xTaskCreate((TaskFunction_t )Start_Task,            	//任务函数
			(const char*    )"Start_Task",                    //任务名称
			(uint16_t       )256,                  						//任务堆栈大小
			(void*          )NULL,                            //传递给任务函数的参数
			(UBaseType_t    )1,                 							//任务优先级
			(TaskHandle_t*  )&Start_Handler);            			//任务句柄  
			vTaskStartScheduler();                            //开启任务调度
}


