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
  * @brief          ���̹�������
  * @param[in]      *chassis_power_limit_f���������ṹ��
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

            chassis_pl_f->chassis_power_pid.SetValue = 19.5f; //��ʹ�ó������ݵ�ʱ��ʹ���ݵ�ѹ�ȶ���19.5f
            increment_pid(&chassis_pl_f->chassis_power_pid, chassis_pl_f->super_cap_c->Capacitance_voltage);

            chassis_pl_f->chassis_speed_gain += -chassis_pl_f->chassis_power_pid.out;

            if (power >= 100 && power <= 120) //�����������ѡ�� gain���������Ʒ�Χ
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
*���ܣ����̷�ֹ�˶��켣ʧ��
*���룺�����ĸ��������ָ��
*�������������ĸ����
*/
/**
  * @brief          ���̷�ֹ�˶��켣ʧ��
  * @param[in]      *v0��
  * @param[in]      *v1��
  * @param[in]      *v2��
  * @param[in]      *v3��
  * @param[in]      SPEED_GAIN��
  * @retval         none
  */
static void keep_driv_track(int16_t *v0, int16_t *v1, int16_t *v2, int16_t *v3, int16_t SPEED_GAIN)
{
    static int16_t max_v = 0;
    static float scale = 1.0f;

    max_v = max_abs(*v0, max_abs(*v1, max_abs(*v2, *v3))); // ȡ�ٶ���ֵ��������
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
  * @brief          �����ʱ�ص�����
  * @param[in]      none
  * @retval         none
  * @attention      ����delay
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
