#include "security_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "rc.h"
#include "init.h"
#include "iwdg.h"
#include "led.h"
#include "delay.h"

int safecheck_heart = 0; //��ȫ�����������

const rc_ctrl_t *rc_security;

uint32_t (*rc_lost_time)(void);

uint8_t safecheck_task_run = 0;
/**
  * @brief          ʧ�ر����������
  * @param[in]      *pvParameters
  * @retval         none
  * @attention
  */
int Offline_time;
void safecheck_task(void *pvParameters)
{
    vTaskDelay(10);

    static portTickType currentTime;

    SECURITY_HEART = 0;

    rc_security = get_remote_control_point();

    currentTime = xTaskGetTickCount(); //��ȡ��ǰϵͳʱ��

    while (1)
    {
        //����vTaskSuspend�����ǲ����ۼƵģ���ʹ��ε��� vTaskSuspend ()������һ���������Ҳֻ�����һ��vTaskResume ()��������ʹ���������������״̬��
        Offline_time = currentTime - rc_lost_time();
        if (currentTime >= rc_lost_time() || rc_data_is_error() == 1) //ң��ʧ��ʱ��̫�� �� ң�������ݴ���
        {
            out_of_control(); //�����������񡢿����쳣��������
        }
        else
        {
            normal_control(); //�ָ���������
        }

        //**B+Shift+Ctrl
        if ((rc_security->kb.bit.B) && (rc_security->kb.bit.CTRL) && (rc_security->kb.bit.SHIFT))
        {
            out_of_control(); //�����������񡢿����쳣��������
            __set_FAULTMASK(1);
            NVIC_SystemReset();
            delay_ms(1000);
        }

        //�������
        vTaskDelayUntil(&currentTime, 50); //������ʱ50ms
    }
}

/**
  * @brief          ʧ�ر�����������
  * @param[in]      *pvParameters
  * @retval         none
  * @attention
  */
void outctl_task(void *pvParameters)
{
    vTaskDelay(5);

    while (1)
    {
        system_out_ctrl_protect();

        vTaskDelay(5);
    }
}

void system_out_ctrl_protect(void) //TODO
{
    GIMBAL_HEART = 1;
    FIRE_HEART = 1;
    SECURITY_HEART = 1;
    WORK_HEART = 1;
    XX_HEART = 1;
    CHASSIS_HEART = 1;
}
