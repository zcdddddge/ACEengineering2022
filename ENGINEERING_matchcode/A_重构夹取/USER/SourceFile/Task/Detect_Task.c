#include "Detect_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "Remote_Task.h"



void Detect_Task(void *pvParameters)
{
    vTaskDelay(500);

    while(1)
    {
        #ifdef UART_RC

        if(Grasp.Remote->RC_ctrl->ch0 >= -660 && Grasp.Remote->RC_ctrl->ch0 <= 660)
        {
            feed ++;
        }

        if(Grasp.Remote->RC_ctrl->ch1 >= -660 && Grasp.Remote->RC_ctrl->ch1 <= 660)
        {
            feed ++;
        }

        if(Grasp.Remote->RC_ctrl->ch3 >= -660 && Grasp.Remote->RC_ctrl->ch3 <= 660)
        {
            feed ++;
        }

        if(Grasp.Remote->RC_ctrl->ch2 >= -660 && Grasp.Remote->RC_ctrl->ch2 <= 660)
        {
            feed ++;
        }

        if(Grasp.Remote->RC_ctrl->sw >= -660 && Grasp.Remote->RC_ctrl->sw <= 660)
        {
            feed ++;
        }

        if(Grasp.Remote->RC_ctrl->s1 >= 1 && Grasp.Remote->RC_ctrl->s1 <= 3)
        {
            feed ++;
        }

        if(Grasp.Remote->RC_ctrl->s2 >= 1 && Grasp.Remote->RC_ctrl->s2 <= 3)
        {
            feed ++;
        }

        if(feed == right || 1)
        {
            IWDG_ReloadCounter();
            feed = 0;
        }

        #endif
//        GPIO_ToggleBits(GPIOE, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
					GPIO_SetBits(GPIOB, GPIO_Pin_7);
        vTaskDelay(200);
    }
}
