#include "shoot_task.h"
#include "rm_motor.h"
#include "chassis_behaviour.h"
#include "led.h"
#include "can_1_receive.h"
#include "referee_deal.h"
#include "system_state.h"
#include "pwm.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* 火控任务 */
#define FIRE_TASK_PRIO 4
#define FIRE_STK_SIZE 128
static TaskHandle_t ShootTask_Handler;

fire_task_t fire_param;

static void fire_control(void);     //发弹系统控制
static void fire_param_init(void);  //参数初始化
void trigger_motor_turn_back(void); //堵转处理

TaskHandle_t *get_shoot_task_handle(void)
{
    return &ShootTask_Handler;
}

void shoot_app_init(void)
{
    //创建火控任务
    xTaskCreate((TaskFunction_t)shoot_task,
                (const char *)"shoot_task",
                (uint16_t)FIRE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)FIRE_TASK_PRIO,
                (TaskHandle_t *)&ShootTask_Handler);
}

//返回火控控制变量，通过指针传递方式传递信息
fire_task_t *get_fire_control_point(void)
{
    return &fire_param;
}

static void snail_motor_pwm(uint32_t snial_pwm)
{
#if (REVERSE_LIGHT_COUPLING == 1)
    if ((snial_pwm >= 1000) && (snial_pwm <= 1500))
        PWM_Shoot_Left = PWM_Shoot_Right = 2111 - snial_pwm;
#else
    if ((snial_pwm >= 1000) && (snial_pwm <= 1500))
        PWM_Shoot_Left = PWM_Shoot_Right = snial_pwm;
#endif
}

/**火控任务**/
void shoot_task(void *pvParameters)
{
    //加载时间
    vTaskDelay(FIRE_TASK_INIT_TIME);

    FIRE_HEART = 0;

    fire_param_init();

    while (1)
    {
        //! 遥控器控制
        if (system_state_return() == INF_RC)
        {
            /* 由底盘主控控制拨弹 */
            {
                if (fire_param.fire_RC->rc.ch[4] >= 500)
                {
                    fire_param.fire_mode = FIRE_H; //拨轮上  开火
					fire_param.fire_switch = 1;
                }
                else if (fire_param.fire_RC->rc.ch[4] <= -500)
                {
                    fire_param.fire_mode = BACK; //拨轮下 反转
                }
                else
                {
                    fire_param.fire_mode = STOP_FIRE; //拨轮中间  停止开火
					fire_param.fire_switch = 0;
                }
            }
            fire_control();
        }
        //! 鼠标键盘控制
        else if (system_state_return() == INF_MK)
        {
            /* 由底盘主控控制拨弹 */
            {
				if (fire_param.fire_mode == STOP_FIRE)
				{
					fire_param.fire_mode = FIRE_L;  //低射频
					fire_param.last_fire_mode = FIRE_L;
				}
				
				if (fire_param.fire_RC->kb.bit.C)
				{
					fire_param.fire_mode = FIRE_L;  //C 低射频
					fire_param.last_fire_mode = FIRE_L;
				}
				else if (fire_param.fire_RC->kb.bit.V)
				{
					fire_param.fire_mode = FIRE_H;  //V 高射频
					fire_param.last_fire_mode = FIRE_H;
				}
				
				if (fire_param.fire_RC->kb.bit.Z) 
                {
                    fire_param.fire_mode = BACK;  //Z   退弹模式(短按即可，长按会烧电机)
                }
				else if (fire_param.last_fire_mode == FIRE_H)
				{
					fire_param.fire_mode = FIRE_H;
				}
				else if (fire_param.last_fire_mode == FIRE_L)
				{
					fire_param.fire_mode = FIRE_L;
				}
				
                if (fire_param.fire_RC->mouse.press_l == 1) //鼠标左键  开火（按住）
                {
					fire_param.fire_switch = 1;
                }
				else
				{
					fire_param.fire_switch = 0;
				}					
            }           
            fire_control();
        }
        //! 停止控制
        else
        {
            fire_task_off();
        }

        vTaskDelay(FIRE_CONTROL_TIME_MS); //2
    }
}

/**
  * @brief          参数初始化
  * @param[in]      none
  * @retval         none
  * @attention  
  */
static void fire_param_init(void)
{
    //获取拨弹电机指针
    fire_param.fire_motor_measure = get_fire_motor_measure_point();
    //获取遥控器指针
    fire_param.fire_RC = get_remote_control_point();

    //射弹模式默认为停火
    fire_param.fire_mode = STOP_FIRE;
    //摩擦轮模式停止模式
    fire_param.friction_mode = STOP_SHOOT;

    pid_init(&fire_param.fire_s_pid, FIRE_S_P, FIRE_S_I, FIRE_S_D, 0, 0);

	fire_param.fire_switch = 0;
    fire_param.shoot_speed = 0;
    fire_param.GD_output = 0;
    snail_motor_pwm(FRICTION_ZERO_SHOOT_SPEED);
    fire_param.GD_speed_gain = 1.0f;
}

void gd_motor_control(uint16_t GD_speed)
{
	if (fire_param.fire_switch == 1) //发射
	{
		//trigger_motor_turn_back(); //堵转处理
		fire_param.GD_set_speed = (GD_speed * fire_param.GD_speed_gain);
		fire_param.GD_output = motor_speed_control(&fire_param.fire_s_pid, fire_param.GD_set_speed, fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
	}
	else
	{
		fire_param.GD_output = motor_speed_control(&fire_param.fire_s_pid, 0, fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
	}
}

/**
  * @brief          发弹系统控制
  * @param[in]      none
  * @retval         none
  * @attention  
  */
static void fire_control(void)
{
    /* 由云台主控控制摩擦轮 */
    if (re_can2_shooter_heat0_speed_limit() != fire_param.shoot_speed)
    {
        fire_param.shoot_speed = re_can2_shooter_heat0_speed_limit();
        if (fire_param.shoot_speed == 15)
        {
            snail_motor_pwm(FRICTION_ZERO_SHOOT_SPEED);
        }
        else if (fire_param.shoot_speed == 18)
        {
            snail_motor_pwm(FRICTION_ONE_SHOOT_SPEED);
        }
        else if (fire_param.shoot_speed == 22)
        {
            snail_motor_pwm(FRICTION_TWO_SHOOT_SPEED);
        }
        else if (fire_param.shoot_speed == 30)
        {
            snail_motor_pwm(FRICTION_THREE_SHOOT_SPEED);
        }
    }

    shoot_hot_limit(); //热量限制

    /* 供弹电机控制 GD=供弹 */
    if (fire_param.fire_mode == STOP_FIRE) //停止发射
    {
        fire_param.GD_output = motor_speed_control(&fire_param.fire_s_pid, 0, fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
	else if (fire_param.fire_mode == FIRE_H)
	{
		gd_motor_control(LOADING_SPEED_H);
	}
	else if (fire_param.fire_mode == FIRE_L)
	{
		gd_motor_control(LOADING_SPEED_L);
	}
	else if (fire_param.fire_mode == BACK) //退弹
    {
        fire_param.GD_output = motor_speed_control(&fire_param.fire_s_pid, -(LOADING_SPEED_L * 0.8f), fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    else
    {
        fire_param.GD_output = motor_speed_control(&fire_param.fire_s_pid, 0, fire_param.fire_motor_measure->speed, M2006_MAX_OUTPUT_CURRENT);
    }
    laser_on();
}

/**
  * @brief          火控系统关闭
  * @param[in]      none
  * @retval         none
  * @attention  
  */
void fire_task_off(void)
{
    snail_motor_pwm(FRICTION_SHOOT_STOP);
    fire_param.GD_output = 0;
    laser_off();
    fire_param.shoot_speed = 0;
    fire_param.GD_speed_gain = 1.0f;
}

/**
  * @brief      热量限制
  * @param[in]  void
  * @retval     通过控制 fire_param.GD_speed_gain 来控制拨弹速度
  *             范围：0% ~ 100%
*/
void shoot_hot_limit(void)
{
    uint16_t Residual_power_percentage;

    fire_param.hot_max = referee_shooter_id1_17mm_cooling_limit();    //机器人 1 号 17mm 枪口热量上限
    fire_param.hot_current = referee_shooter_id1_17mm_cooling_heat(); //1 号 17mm 枪口热量

    Residual_power_percentage = (fire_param.hot_max - fire_param.hot_current);

    if (Residual_power_percentage > 0 && Residual_power_percentage < 40)
    {
        //做热量限制
        fire_param.GD_speed_gain = (float)(Residual_power_percentage / 40.0f) * (Residual_power_percentage / 40.0f);
    }
    else
    {
        //不做热量限制
        fire_param.GD_speed_gain = 1.0f;
    }
}

/**
  * @brief      堵转处理
  * @param[in]  void
  * @retval     
*/
void trigger_motor_turn_back(void) //堵转处理
{
    if (fire_param.block_time >= BLOCK_TIME)
    {
        fire_param.fire_mode = BACK;
    }

    if (int16_t_abs(fire_param.fire_motor_measure->speed) < BLOCK_TRIGGER_SPEED && fire_param.block_time < BLOCK_TIME)
    {
        fire_param.block_time++; //BLOCK_TIME：堵转时间  达到这个时间开始反转
        fire_param.reverse_time = 0;
    }
    else if (fire_param.block_time == BLOCK_TIME && fire_param.reverse_time < REVERSE_TIME)
    {
        fire_param.reverse_time++; //REVERSE_TIME：反转时间
    }
    else
    {
        fire_param.block_time = 0;
    }
}

///**
//  * @brief      射击控制，控制拨弹电机角度，完成一次发射
//  * @param[in]  void
//  * @retval     void
//  */
//static void shoot_bullet_control(void)
//{

//    //每次拨动 1/4PI的角度
//    if (shoot_control.move_flag == 0)
//    {
//        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
//        shoot_control.move_flag = 1;
//    }
//    if (shoot_control.key == SWITCH_TRIGGER_OFF)
//    {
//        shoot_control.shoot_mode = SHOOT_DONE;
//    }
//    //到达角度判断
//    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
//    {
//        //没到达一直设置旋转速度
//        shoot_control.trigger_speed_set = TRIGGER_SPEED;
//        trigger_motor_turn_back();
//    }
//    else
//    {
//        shoot_control.move_flag = 0;
//    }
//}


uint16_t re_shoot_status(void)
{
	return (fire_param.fire_mode);
}






