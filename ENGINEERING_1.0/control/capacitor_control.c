#include "capacitor_control.h"
#include "rm_motor.h"
#include "maths.h"
#include "chassis_task.h"
#include "referee_deal.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "chassis_app.h"

#pragma diag_suppress 177


static void keep_driv_track(int16_t *v0, int16_t *v1, int16_t *v2, int16_t *v3, int16_t SPEED_GAIN);

/**
  * @brief          底盘功率限制
  * @param[in]      *chassis_power_limit_f：底盘主结构体
  * @retval         none
  */
extern Supercapacitor_receive_t Supercap_receive;
void chassis_power_limit_control(chassis_control_t *chassis_pl_f)
{
    if (chassis_pl_f->SuperCap_discharge_flag == 1) //TODO
    {
        chassis_pl_f->chassis_speed_gain = (referee_chassis_power_limit() / 5.0f) * 1.1f;
    }
    else
    {
        if (chassis_pl_f->super_cap_c->Capacitance_voltage > 12.0f)
        {
            uint16_t power = referee_chassis_power_limit();

            chassis_pl_f->chassis_power_pid.SetValue = 19.5f; //不使用超级电容的时候，使电容电压稳定在19.5f
            increment_pid(&chassis_pl_f->chassis_power_pid, chassis_pl_f->super_cap_c->Capacitance_voltage);

            chassis_pl_f->chassis_speed_gain += -chassis_pl_f->chassis_power_pid.out;

            if (power >= 100 && power <= 120) //根据最大功率来选择 gain参数的限制范围
            {
                chassis_pl_f->chassis_speed_gain = float_limit(chassis_pl_f->chassis_speed_gain, 24.0f, 10.0f);
            }
            else
            {
                chassis_pl_f->chassis_speed_gain = float_limit(chassis_pl_f->chassis_speed_gain, 18.0f, 3.0f);
            }

            chassis_pl_f->chassis_speed_gain = (1 - 0.04f) * chassis_pl_f->chassis_last_speed_gain + (0.04f) * chassis_pl_f->chassis_speed_gain;
            chassis_pl_f->chassis_last_speed_gain = chassis_pl_f->chassis_speed_gain;
        }
        else if (Supercap_receive.Capacitance_voltage == 0.0f)
        {
            chassis_pl_f->chassis_speed_gain = 8.0f;
        }
    }
}

/*
*功能：底盘防止运动轨迹失真
*传入：底盘四个轮子输出指针
*传出：处理后的四个输出
*/
/**
  * @brief          底盘防止运动轨迹失真
  * @param[in]      *v0：
  * @param[in]      *v1：
  * @param[in]      *v2：
  * @param[in]      *v3：
  * @param[in]      SPEED_GAIN：
  * @retval         none
  */
static void keep_driv_track(int16_t *v0, int16_t *v1, int16_t *v2, int16_t *v3, int16_t SPEED_GAIN)
{
    static int16_t max_v = 0;
    static float scale = 1.0f;

    max_v = max_abs(*v0, max_abs(*v1, max_abs(*v2, *v3))); // 取速度数值最大的轮子
    if (SPEED_GAIN == 0)
    {
        *v0 = *v1 = *v2 = *v3 = 0;
    }
    else if (max_v > (SPEED_GAIN * 660))
    {
        scale = max_v / (SPEED_GAIN * 660);
        *v0 = (int16_t)((*v0) / scale);
        *v1 = (int16_t)((*v1) / scale);
        *v2 = (int16_t)((*v2) / scale);
        *v3 = (int16_t)((*v3) / scale);
    }
}

/**
  * @brief          软件定时回调函数
  * @param[in]      none
  * @retval         none
  * @attention      禁用delay
  */
void cap_timer_callback(TimerHandle_t xTimer)
{
    uint16_t power = referee_chassis_power_limit();
    if (power >= 40 && power <= 120)
    {
        can1_cap_setmsg(power);
    }
    else
    {
        can1_cap_setmsg(80);
    }
}
