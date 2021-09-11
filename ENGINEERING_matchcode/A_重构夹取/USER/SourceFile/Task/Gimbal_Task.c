#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"

void Gimbal_Task(void *pvParameters)
{
    vTaskDelay(500);

    while(1)
    {

        vTaskDelay(2);
    }
}
