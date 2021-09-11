#include "timer_task.h"
#include "capacitor_control.h"
#include "referee_deal.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*
在FreeRTOSConfig.h文件中使能宏定义：
#define configUSE_TIMERS  1
*/
TimerHandle_t CAP_Timer_Handle; //周期定时器句柄
TimerHandle_t Referee_Timer_Handle;

void timer_send_create(void)
{
    taskENTER_CRITICAL(); //进入临界区

    //创建can1发送定时器
    CAP_Timer_Handle = xTimerCreate((const char *)"CAP_Timer",
                                    (TickType_t)100,                              //10Hz - 100ms一次
                                    (UBaseType_t)pdTRUE,                          //周期执行
                                    (void *)0,                                    //定时器ID
                                    (TimerCallbackFunction_t)cap_timer_callback); //回调函数

    //开启定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
    if (CAP_Timer_Handle != NULL)
    {
        xTimerStart(CAP_Timer_Handle, 0); //不等待
    }

    taskEXIT_CRITICAL(); //退出临界区
}


void timer_receive_create(void)
{
    taskENTER_CRITICAL(); //进入临界区

    //创建can1发送定时器
    Referee_Timer_Handle = xTimerCreate((const char *)"Referee_Timer",
                                        (TickType_t)10,                              //100Hz - 10ms一次
                                        (UBaseType_t)pdTRUE,                          //周期执行
                                        (void *)1,                                    //定时器ID
                                        (TimerCallbackFunction_t)referee_read_data);  //回调函数

    //开启定时器,仅且仅能开启一次，否则会出错，不开启则不会发数据
    if (Referee_Timer_Handle != NULL)
    {
        xTimerStart(Referee_Timer_Handle, 0); //不等待
    }

    taskEXIT_CRITICAL(); //退出临界区
}



