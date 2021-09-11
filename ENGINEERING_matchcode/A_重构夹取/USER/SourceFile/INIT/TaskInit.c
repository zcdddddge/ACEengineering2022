#include "TaskInit.h"
#include "Detect_Task.h"
#include "Grasp_Task.h"
#include "Remote_Task.h"
#include "Data_Task.h"


	
/*遥控任务*/
TaskHandle_t 					Remote_Handler;
#define Remote_Size		256
#define Remote_Prio		7


/*检测任务*/
TaskHandle_t 					Detect_Handler;
#define Detect_Size		256
#define Detect_Prio		7

/*夹取任务*/
TaskHandle_t 					Grasp_Handler;
#define Grasp_Size		256
#define Grasp_Prio		8

/*数据任务*/
TaskHandle_t 					Data_Handler;
#define Data_Size			256
#define Data_Prio			8


/*任务初始化*/
void TaskInit(void)
{
		#if 1
		xTaskCreate((TaskFunction_t )Remote_Task,            	
			(const char*    )"Remote_Task",                  
			(uint16_t       )Remote_Size,                           
			(void*          )NULL,                          
			(UBaseType_t    )Remote_Prio,               
			(TaskHandle_t*  )&Remote_Handler);
		#endif
			
		#if 1
		xTaskCreate((TaskFunction_t )Detect_Task,            	
			(const char*    )"Detect_Task",                  
			(uint16_t       )Detect_Size,                           
			(void*          )NULL,                          
			(UBaseType_t    )Detect_Prio,               
			(TaskHandle_t*  )&Detect_Handler);
		#endif
			
		#if 1
		xTaskCreate((TaskFunction_t )Data_Task,            	
			(const char*    )"Data_Task",                  
			(uint16_t       )Data_Size,                           
			(void*          )NULL,                          
			(UBaseType_t    )Data_Prio,               
			(TaskHandle_t*  )&Data_Handler);
		#endif
		
		#if 1
		xTaskCreate((TaskFunction_t )Grasp_Task,            	
			(const char*    )"Grasp_Task",                  
			(uint16_t       )Grasp_Size,                           
			(void*          )NULL,                          
			(UBaseType_t    )Grasp_Prio,               
			(TaskHandle_t*  )&Grasp_Handler);
		#endif
		
}
