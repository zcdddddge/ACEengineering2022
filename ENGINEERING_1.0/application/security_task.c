#include "security_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "rc.h"
#include "init.h"
#include "iwdg.h"
#include "led.h"
#include "delay.h"

int safecheck_heart = 0; //安全检测任务心跳

const rc_ctrl_t *rc_security;

uint32_t (*rc_lost_time)(void);

uint8_t safecheck_task_run = 0;
/**
  * @brief          失控保护检测任务
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

    currentTime = xTaskGetTickCount(); //获取当前系统时间

    while (1)
    {
        //调用vTaskSuspend函数是不会累计的：即使多次调用 vTaskSuspend ()函数将一个任务挂起，也只需调用一次vTaskResume ()函数就能使挂起的任务解除挂起状态。
        Offline_time = currentTime - rc_lost_time();
        if (currentTime >= rc_lost_time() || rc_data_is_error() == 1) //遥控失联时间太长 或 遥控器数据错误
        {
            out_of_control(); //挂起正常任务、开启异常处理任务
        }
        else
        {
            normal_control(); //恢复正常任务
        }

        //**B+Shift+Ctrl
        if ((rc_security->kb.bit.B) && (rc_security->kb.bit.CTRL) && (rc_security->kb.bit.SHIFT))
        {
            out_of_control(); //挂起正常任务、开启异常处理任务
            __set_FAULTMASK(1);
            NVIC_SystemReset();
            delay_ms(1000);
        }

        //检测周期
        vTaskDelayUntil(&currentTime, 50); //绝对延时50ms
    }
}

/**
  * @brief          失控保护控制任务
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
