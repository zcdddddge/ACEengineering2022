#include "Data_Task.h"
#include "GraspMotor.h"
#include "BoardCommuni.h"

#include "FreeRTOS.h"
#include "task.h"



void Data_Task(void *pvParameters)
{
    vTaskDelay(500);
    Board_Communi_Init();


    while(1)
    {
        Update_Sensor_Val();
        Update_VL53L0_Val();
        Board_Communi_Updata();
        vTaskDelay(2);
    }
}
