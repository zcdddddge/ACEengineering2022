#include "TaskInit.h"
#include "Gimbal_Task.h"
#include "Detect_Task.h"
#include "Chassis_Task.h"
#include "Remote_Task.h"
#include "Communi_Task.h"

/*底盘任务*/
TaskHandle_t 					Chassis_Handler;
#define Chassis_Size	256
#define Chassis_Prio	6
	
/*遥控任务*/
TaskHandle_t 					Remote_Handler;
#define Remote_Size		256
#define Remote_Prio		8

/*云台任务*/
TaskHandle_t 					Gimbal_Handler;
#define Gimbal_Size		256
#define Gimbal_Prio		7

/*检测任务*/
TaskHandle_t 					Detect_Handler;
#define Detect_Size		256
#define Detect_Prio		11

/*检测任务*/
TaskHandle_t 					Communi_Handler;
#define Communi_Size	256
#define Communi_Prio	9



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
		xTaskCreate((TaskFunction_t )Chassis_Task,            	
			(const char*    )"Chassis_Task",                  
			(uint16_t       )Chassis_Size,                           
			(void*          )NULL,                          
			(UBaseType_t    )Chassis_Prio,               
			(TaskHandle_t*  )&Chassis_Handler);
		#endif
			
		#if 1
		xTaskCreate((TaskFunction_t )Gimbal_Task,			
			(const char*    )"Gimbal_Task",                  
			(uint16_t       )Gimbal_Size,                           
			(void*          )NULL,                          
			(UBaseType_t    )Gimbal_Prio,               
			(TaskHandle_t*  )&Gimbal_Handler);
		#endif
			
		#if 1
		xTaskCreate((TaskFunction_t )Detect_Task,            	
			(const char*    )"Detect_Task",                  
			(uint16_t       )Detect_Size,                           
			(void*          )NULL,                          
			(UBaseType_t    )Detect_Prio,               
			(TaskHandle_t*  )&Detect_Handler);
		#endif
			
			
		#if 0
		xTaskCreate((TaskFunction_t )Communi_Task,            	
			(const char*    )"Communi_Task",                  
			(uint16_t       )Communi_Size,                           
			(void*          )NULL,                          
			(UBaseType_t    )Communi_Prio,               
			(TaskHandle_t*  )&Communi_Handler);
		#endif
}
