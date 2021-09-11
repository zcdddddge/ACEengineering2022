#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"

G_t  G ;

void Gimbal_Task(void *pvParameters)
{
    vTaskDelay(500);
    PY_Motor_Init(&G);

    while(1)
    {
        PY_Motor(&G);
        vTaskDelay(2);
    }
}
