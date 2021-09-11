#include "timer_task.h"
#include "capacitor_control.h"
#include "referee_deal.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*
��FreeRTOSConfig.h�ļ���ʹ�ܺ궨�壺
#define configUSE_TIMERS  1
*/
TimerHandle_t CAP_Timer_Handle; //���ڶ�ʱ�����
TimerHandle_t Referee_Timer_Handle;

void timer_send_create(void)
{
    taskENTER_CRITICAL(); //�����ٽ���

    //����can1���Ͷ�ʱ��
    CAP_Timer_Handle = xTimerCreate((const char *)"CAP_Timer",
                                    (TickType_t)100,                              //10Hz - 100msһ��
                                    (UBaseType_t)pdTRUE,                          //����ִ��
                                    (void *)0,                                    //��ʱ��ID
                                    (TimerCallbackFunction_t)cap_timer_callback); //�ص�����

    //������ʱ��,���ҽ��ܿ���һ�Σ����������������򲻻ᷢ����
    if (CAP_Timer_Handle != NULL)
    {
        xTimerStart(CAP_Timer_Handle, 0); //���ȴ�
    }

    taskEXIT_CRITICAL(); //�˳��ٽ���
}


void timer_receive_create(void)
{
    taskENTER_CRITICAL(); //�����ٽ���

    //����can1���Ͷ�ʱ��
    Referee_Timer_Handle = xTimerCreate((const char *)"Referee_Timer",
                                        (TickType_t)10,                              //100Hz - 10msһ��
                                        (UBaseType_t)pdTRUE,                          //����ִ��
                                        (void *)1,                                    //��ʱ��ID
                                        (TimerCallbackFunction_t)referee_read_data);  //�ص�����

    //������ʱ��,���ҽ��ܿ���һ�Σ����������������򲻻ᷢ����
    if (Referee_Timer_Handle != NULL)
    {
        xTimerStart(Referee_Timer_Handle, 0); //���ȴ�
    }

    taskEXIT_CRITICAL(); //�˳��ٽ���
}



