#include "Remote_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ChassisMotor.h"

void Remote_Task(void *pvParameters)
{
    vTaskDelay(500);
    Remote_Data_Init();

    while(1)
    {
        Remote_Data_Deal();
		

        vTaskDelay(2);
    }
}
