#include "Detect_Task.h"
#include "Chassis_Task.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"

extern Chassis_t Chassis;
#define right 7
int16_t feed = 0;

void Detect_Task(void *pvParameters)
{
    vTaskDelay(200);
    while(1)
    {


        if(Chassis.RC->RC_ctrl->ch0 >= -660 && Chassis.RC->RC_ctrl->ch0 <= 660)
        {
            feed ++;
        }

        if(Chassis.RC->RC_ctrl->ch1 >= -660 && Chassis.RC->RC_ctrl->ch1 <= 660)
        {
            feed ++;
        }

        if(Chassis.RC->RC_ctrl->ch3 >= -660 && Chassis.RC->RC_ctrl->ch3 <= 660)
        {
            feed ++;
        }

        if(Chassis.RC->RC_ctrl->ch2 >= -660 && Chassis.RC->RC_ctrl->ch2 <= 660)
        {
            feed ++;
        }

        if(Chassis.RC->RC_ctrl->sw >= -660 && Chassis.RC->RC_ctrl->sw <= 660)
        {
            feed ++;
        }

        if(Chassis.RC->RC_ctrl->s1 >= 1 && Chassis.RC->RC_ctrl->s1 <= 3)
        {
            feed ++;
        }

        if(Chassis.RC->RC_ctrl->s2 >= 1 && Chassis.RC->RC_ctrl->s2 <= 3)
        {
            feed ++;
        }

        if(feed == right)
        {
            IWDG_ReloadCounter();
            feed = 0;
        }


        GPIO_ToggleBits(GPIOE, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
        GPIO_ToggleBits(GPIOB, GPIO_Pin_7);
        vTaskDelay(200);
    }
}
