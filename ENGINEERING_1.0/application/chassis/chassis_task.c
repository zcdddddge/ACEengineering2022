/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       chassis_task.c/h
  * @brief      ��ɵ��̿�������
  * @note       �ϲ��汾
  * @history    v5.0
  *
  @verbatim   
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  */
#include "chassis_task.h"
#include "chassis_app.h"
#include "chassis_behaviour.h"
#include "shoot_task.h"
#include "maths.h"
#include "rm_motor.h"
#include "filter.h"
#include "can_1_receive.h"
#include "can_2_receive.h"
#include "photoelectric.h"
#include "detect_task.h"
#include "capacitor_control.h"
#include "rc.h"
#include "imu.h"
#include "iwdg.h"
#include "led.h"
#include "system_state.h"

#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "referee_deal.h"

/*--------------------����--------------------*/
//���̿������� static
chassis_control_t chassis_control;
static const chassis_behaviour_e *chassis_beh;

#if CHASSIS_TEST_MODE
int JSCOPE_chassis_1;
int JSCOPE_chassis_2;
int JSCOPE_chassis_3;
int JSCOPE_chassis_4;
int temp_1, temp_2, Can_send_time;
#endif

/*--------------------����--------------------*/
static void chassis_to_gimbal(chassis_control_t *chassis_setmsg_f);                //���̷������ݵ���̨��yaw������������ֵ��yaw�����ٶ�ֵ��ң��ֵ��
static void chassis_controlwork(chassis_control_t *chassis_control_f);             //��Ҫ���ƺ��� ѭ��
static void chassis_init(chassis_control_t *chassis_move_init_f);                  //�������ݳ�ʼ��
static void chassis_data_update(chassis_control_t *chassis_data_update_f);         //�������ݸ���
static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f);  //����ң��ģʽѡ��
static void chassis_pid_calc(chassis_control_t *chassis_pid_f);                    //����pid����
static void chassis_accelerated_control(int16_t *ch0, int16_t *ch1, int16_t *ch2); //���ٶ����ƴ���
#if CHASSIS_TEST_MODE
static void chassis_jscope_print_curve(void); /* jscope��ӡ���� */
#endif


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_task_stack;
#endif

/**
  * @brief          ���̿���������
  * @param[in]      none
  * @retval         none
  */
void chassis_task(void *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //�������ݳ�ʼ��
    chassis_init(&chassis_control);

    CHASSIS_HEART = 0;

    while (1)
    {		
        //*���̿���
        chassis_controlwork(&chassis_control);

        taskENTER_CRITICAL(); //�����ٽ���
		//*can2���̷����ݵ���̨
        chassis_to_gimbal(&chassis_control);

        //*can1���͵���ֵ
        can1_chassis_setmsg(chassis_control.chassis_motor[0].give_current,
                            chassis_control.chassis_motor[1].give_current,
                            chassis_control.chassis_motor[2].give_current,
                            chassis_control.chassis_motor[3].give_current);
//        can1_chassis_gimbal_fire(chassis_control.fire_c->GD_output, 0, chassis_control.fire_c->GD_output); //���й���2006����Ŀ���
			
				can1_syaw_setmsg(chassis_control.yaw_6020_c->yaw_6020_output);//С��̨
				can1_spitch_setmsg(chassis_control.pitch_2006_c->pitch_2006_output);
			
				can1_rescue_setmsg(chassis_control.rescue_motor_c->rescue_motor_output,chassis_control.rescue_clap_c->rescue_clap_output);
			
			
        taskEXIT_CRITICAL();                                                                               //�˳��ٽ���

        vTaskDelay(CHASSIS_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_task_stack = uxTaskGetStackHighWaterMark(NULL); //��ջ����ʷʣ����Сֵ
#endif
    }
}

/**
  * @brief          ���̷������ݵ���̨��yaw������������ֵ��yaw�����ٶ�ֵ��ң��ֵ��
  * @param[in]      *chassis_setmsg_f���������ṹ��
  * @retval         none
  */
static int send_sign = 0;
static void chassis_to_gimbal(chassis_control_t *chassis_setmsg_f)
{
    if (send_sign == 0)
    {
        if (chassis_setmsg_f->chassis_RC->rc.s2 == RC_SW_MID && chassis_setmsg_f->chassis_RC->rc.s1 == RC_SW_MID)
            can2_chassis_mk_setmsg(chassis_setmsg_f->chassis_RC); //s1��s2�������м�Ϊ����ģʽ
        else
            can2_chassis_rc_setmsg(chassis_setmsg_f->chassis_RC); //����Ϊң����ģʽ
        send_sign = 1;
    }
    if (send_sign == 1)
    {
        can2_4002_send();
        send_sign = 0;
    }
}

/**
  * @brief          �������ݳ�ʼ��
  * @param[in]      *chassis_move_init_f���������ṹ��
  * @retval         none
  */
static void chassis_init(chassis_control_t *chassis_move_init_f)
{
    uint8_t i;

    /*--------------------��ȡָ��--------------------*/
    {
        //����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
        chassis_move_init_f->chassis_RC = get_remote_control_point();
        //��ȡ����ָ��
        for (i = 0; i < 4; i++)
        {
            chassis_move_init_f->chassis_motor[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        }
				
				chassis_move_init_f->yaw_6020_c->yaw6020_measure = get_syaw_motor_measure_point();
				chassis_move_init_f->pitch_2006_c->pitch2006_measure = get_spitch_motor_measure_point();
				
				chassis_move_init_f->rescue_motor_c->rescue_motor_measure = get_rescue_motor_measure_point();
				chassis_move_init_f->rescue_clap_c->rescue_clap_measure = get_rescue_clap_motor_measure_point();
				
				
        //������̨��can2�����������ݽṹ��ָ��
        chassis_move_init_f->gimbal_re_data = get_yaw_receive_measure_point();
				
				chassis_move_init_f->chassis_re_sensorL = re_chassis_gimbal_sensorL();
				chassis_move_init_f->chassis_re_sensorR = re_chassis_gimbal_sensorR();
        //��ȡcan1��yaw���ָ��
        //chassis_move_init_f->yaw_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();

        chassis_move_init_f->fire_c = get_fire_control_point();
        chassis_move_init_f->super_cap_c = get_supercap_control_point();
        chassis_beh = get_chassis_behaviour_point();
				
				chassis_move_init_f->gyro = Return_PYR_t();
    }

    /*--------------------��ʼ����ͨ�˲�--------------------*/
    {
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vx), CHASSIS_FIRST_ORDER_FILTER_K);
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vy), CHASSIS_FIRST_ORDER_FILTER_K);
    }

    /*--------------------��ʼ������pid--------------------*/
    {
        //�����ƶ�pid
        pid_init(&chassis_move_init_f->chassis_speed_pid[0], CHASSIS_MOTOR1_PID_Kp, CHASSIS_MOTOR1_PID_Ki, CHASSIS_MOTOR1_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[1], CHASSIS_MOTOR2_PID_Kp, CHASSIS_MOTOR2_PID_Ki, CHASSIS_MOTOR2_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[2], CHASSIS_MOTOR3_PID_Kp, CHASSIS_MOTOR3_PID_Ki, CHASSIS_MOTOR3_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[3], CHASSIS_MOTOR4_PID_Kp, CHASSIS_MOTOR4_PID_Ki, CHASSIS_MOTOR4_PID_Kd, 0, 0);
        //����λ�û�pid
        pid_init(&chassis_move_init_f->chassis_location_pid, CHASSIS_LOCATION_PID_P, CHASSIS_LOCATION_PID_I, CHASSIS_LOCATION_PID_D, 0, 0);
        //������ת����pid
        pid_init(&chassis_move_init_f->chassis_rotate_pid, CHASSIS_ROTATE_FOLLOW_P, CHASSIS_ROTATE_FOLLOW_I, CHASSIS_ROTATE_FOLLOW_D, 0, 0);
        chassis_move_init_f->chassis_rotate_pid.errorabsmin = 0.5f;
        chassis_move_init_f->chassis_rotate_pid.errorabsmax = 180.0f;
        chassis_move_init_f->chassis_rotate_pid.deadband = 0.0f;

        pid_init(&chassis_move_init_f->chassis_power_pid, 10.0f, 1.0f, 20.5f, 0, 0);
        chassis_move_init_f->chassis_power_pid.errorabsmin = 0.0f;
        chassis_move_init_f->chassis_power_pid.errorabsmax = 5.0f;
			
				pid_init(&chassis_move_init_f->pitch_2006_c->pitch_2006_pid_p,SGIMBAL_PITCH_P_P,SGIMBAL_PITCH_P_I,SGIMBAL_PITCH_P_D,0,0);
				pid_init(&chassis_move_init_f->pitch_2006_c->pitch_2006_pid_s,SGIMBAL_PITCH_S_P,SGIMBAL_PITCH_S_I,SGIMBAL_PITCH_S_D,0,0);
			
				pid_init(&chassis_move_init_f->yaw_6020_c->yaw_6020_pid_p,SGIMBAL_YAW_P_P,SGIMBAL_YAW_P_I,SGIMBAL_YAW_P_D,0,0);
				pid_init(&chassis_move_init_f->yaw_6020_c->yaw_6020_pid_s,SGIMBAL_YAW_S_P,SGIMBAL_YAW_S_I,SGIMBAL_YAW_S_D,0,0);
				
				pid_init(&chassis_move_init_f->rescue_motor_c->rescue_motor_pid_s,RESCUE_S_P,RESCUE_S_I,RESCUE_S_D,0,0);
				pid_init(&chassis_move_init_f->rescue_clap_c->rescue_clap_pid_s,RESCUECLAP_S_P,RESCUECLAP_S_I,RESCUECLAP_S_D,0,0);
    }
		
    //���̿���״̬Ϊֹͣ (ע������ڻ�ȡָ���Ժ�)
    chassis_task_off(1);

    //���ν����������
    chassis_data_update(chassis_move_init_f);
}

/**
  * @brief          ������Ҫ������ƺ���
  * @param[in]      *chassis_control_f���������ṹ��
  * @retval         none
  */
static void chassis_controlwork(chassis_control_t *chassis_control_f)
{
    //�����������̨���
    chassis_control_f->chassis_gimbal_angel = chassis_control_f->gimbal_re_data->chassis_gimbal_angel; //-Get_Yaw_Different_Angle(chassis_control_f->yaw_motor_measure, YAW_RATIO);
    //����̨��Ե��̵Ĳ��
    //sec_gimbal_control.chassis_different_angle = Sec_chassis_angle;

    //�������ݸ���
    chassis_data_update(chassis_control_f);

    //ң����ģʽ״̬����
    chassis_remote_mode_choose(chassis_control_f);

    //���̿���PID����
    chassis_pid_calc(chassis_control_f);
}

/**
  * @brief          �������ݸ���
  * @param[in]      *chassis_data_update_f���������ṹ��
  * @retval         none
  * @attention      
  */
static void chassis_data_update(chassis_control_t *chassis_data_update_f)
{
    //����ٶȸ���
    for (uint16_t i = 0; i < 4; i++)
    {
        chassis_data_update_f->chassis_motor[i].speed = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->speed;
        chassis_data_update_f->chassis_motor[i].position = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->position;
        chassis_data_update_f->chassis_motor[i].accel = chassis_data_update_f->chassis_speed_pid[i].Derror[0] * 500.0f;
    }
        chassis_data_update_f->yaw_6020_c->speed = chassis_data_update_f->yaw_6020_c->yaw6020_measure->speed;
        chassis_data_update_f->yaw_6020_c->position = chassis_data_update_f->yaw_6020_c->yaw6020_measure->position;
        chassis_data_update_f->pitch_2006_c->speed = chassis_data_update_f->pitch_2006_c->pitch2006_measure->speed;
        chassis_data_update_f->pitch_2006_c->position = chassis_data_update_f->pitch_2006_c->pitch2006_measure->position;
		
        chassis_data_update_f->rescue_motor_c->speed = chassis_data_update_f->rescue_motor_c->rescue_motor_measure->speed;
        chassis_data_update_f->rescue_motor_c->position = chassis_data_update_f->rescue_motor_c->rescue_motor_measure->position;
				chassis_data_update_f->rescue_motor_c->accel = chassis_data_update_f->rescue_motor_c->rescue_motor_pid.Derror[0] * 500.0f;
		
        chassis_data_update_f->rescue_clap_c->speed = chassis_data_update_f->rescue_clap_c->rescue_clap_measure->speed;
        chassis_data_update_f->rescue_clap_c->position = chassis_data_update_f->rescue_clap_c->rescue_clap_measure->position;
				chassis_data_update_f->rescue_clap_c->accel = chassis_data_update_f->rescue_clap_c->rescue_clap_pid.Derror[0] * 500.0f;
	
}

/**
  * @brief          ����ң��ģʽѡ��
  * @param[in]      *chassis_mode_choose_f���������ṹ��
  * @retval         none
  */
static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f)
{
    //�ж���ʲôģʽ
    chassis_behaviour_mode_set(chassis_mode_choose_f);
}

/**
  * @brief          ���̿���������
  * @param[in]      *chassis_set_f���������ṹ��
  *                 ch0��������ch0����ֵ
  *                 ch1��������ch1����ֵ
  *                 ch2��������ch2����ֵ
  * @retval         none
  */
void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2)
{
#ifdef POWER_LIMIT //��������
    chassis_power_limit_control(chassis_set_f);
#endif

    /*���������ݵ���ģʽ����*/
    if (system_state_return() == INF_STOP)
    {
        //һ�׵�ͨ�˲�����
        first_order_filter(&(chassis_set_f->LowFilt_chassis_vx), -ch0);
        first_order_filter(&(chassis_set_f->LowFilt_chassis_vy), ch1);

        chassis_set_f->speed_x_set = 0;
        chassis_set_f->speed_y_set = 0;
        chassis_set_f->speed_z_set = 0;
        chassis_set_f->chassis_motor[0].speed_set = 0;
        chassis_set_f->chassis_motor[1].speed_set = 0;
        chassis_set_f->chassis_motor[2].speed_set = 0;
        chassis_set_f->chassis_motor[3].speed_set = 0;
        chassis_set_f->chassis_motor[0].give_current = 0;
        chassis_set_f->chassis_motor[1].give_current = 0;
        chassis_set_f->chassis_motor[2].give_current = 0;
        chassis_set_f->chassis_motor[3].give_current = 0;
    }
    else
    {
        //���� ��������̨ | Ť�� | С���� ��������ʱ������������������x��y�����z����ȥ���ˣ�
        if (*chassis_beh == CHASSIS_NO_FOLLOW || *chassis_beh == CHASSIS_TWIST_WAIST || *chassis_beh == CHASSIS_ROTATION)
        {
            chassis_set_f->LowFilt_chassis_vx.out = 0;
            chassis_set_f->LowFilt_chassis_vy.out = 0;

            chassis_set_f->speed_x_set = -ch0;
            chassis_set_f->speed_y_set = ch1;
            chassis_set_f->speed_z_set = ch2;

            //�����˶��ֽ�
            chassis_set_f->chassis_motor[0].speed_set = (-chassis_set_f->speed_y_set - chassis_set_f->speed_x_set + chassis_set_f->speed_z_set) * chassis_set_f->chassis_speed_gain * 0.9f; //5.0f
            chassis_set_f->chassis_motor[1].speed_set = (chassis_set_f->speed_y_set - chassis_set_f->speed_x_set + chassis_set_f->speed_z_set) * chassis_set_f->chassis_speed_gain * 0.9f;
            chassis_set_f->chassis_motor[2].speed_set = (-chassis_set_f->speed_y_set + chassis_set_f->speed_x_set + chassis_set_f->speed_z_set) * chassis_set_f->chassis_speed_gain * 0.9f;
            chassis_set_f->chassis_motor[3].speed_set = (chassis_set_f->speed_y_set + chassis_set_f->speed_x_set + chassis_set_f->speed_z_set) * chassis_set_f->chassis_speed_gain * 0.9f;
        }
        //���� ������̨
        else if (*chassis_beh == CHASSIS_FOLLOW)
        {
            /*������б�º�������*/
            chassis_accelerated_control(&ch0, &ch1, &ch2);

            //һ�׵�ͨ�˲�����
            first_order_filter(&(chassis_set_f->LowFilt_chassis_vx), -ch0);
            first_order_filter(&(chassis_set_f->LowFilt_chassis_vy), ch1);

            if (ch0 == 0.0f)
            {
                chassis_set_f->LowFilt_chassis_vx.out = 0.0f;
            }
            if (ch1 == 0.0f)
            {
                chassis_set_f->LowFilt_chassis_vy.out = 0.0f;
            }

            chassis_set_f->speed_x_set = chassis_set_f->LowFilt_chassis_vx.out;
            chassis_set_f->speed_y_set = chassis_set_f->LowFilt_chassis_vy.out;
            chassis_set_f->speed_z_set = ch2;

            //�����˶��ֽ�
            chassis_set_f->chassis_motor[0].speed_set = (-chassis_set_f->speed_y_set - chassis_set_f->speed_x_set + chassis_set_f->speed_z_set) * chassis_set_f->chassis_speed_gain; //15.0f  8.0f
            chassis_set_f->chassis_motor[1].speed_set = (chassis_set_f->speed_y_set - chassis_set_f->speed_x_set + chassis_set_f->speed_z_set) * chassis_set_f->chassis_speed_gain;  //8.0f
            chassis_set_f->chassis_motor[2].speed_set = (-chassis_set_f->speed_y_set + chassis_set_f->speed_x_set + chassis_set_f->speed_z_set) * chassis_set_f->chassis_speed_gain;
            chassis_set_f->chassis_motor[3].speed_set = (chassis_set_f->speed_y_set + chassis_set_f->speed_x_set + chassis_set_f->speed_z_set) * chassis_set_f->chassis_speed_gain;
        }
    }

#if CHASSIS_TEST_MODE
    chassis_jscope_print_curve(); // jscope��ӡ����
#endif
}
/**
  * @brief          С��̨����������
  * @param[in]      *chassis_set_f���������ṹ��
  *                 ch0��������ch0����ֵ
  *                 ch1��������ch1����ֵ
  * @retval         none
  */
void sgimbal_set_remote(chassis_control_t *sgimabl_set_f, int16_t ch0, int16_t ch1)
{
		sgimabl_set_f->yaw_6020_c->yaw_6020_output = motor_position_speed_control(&sgimabl_set_f->yaw_6020_c->yaw_6020_pid_s,
																																			&sgimabl_set_f->yaw_6020_c->yaw_6020_pid_p,
																																			0,
																																			sgimabl_set_f->yaw_6020_c->speed,
																																			loop_fp32_constrain((ch0 + 257.0f - (sgimabl_set_f->yaw_6020_c->yaw6020_measure->actual_Position*360/8192)), -180.0f, 180.0f),
																																			11000);
		sgimabl_set_f->pitch_2006_c->pitch_2006_output = motor_position_speed_control(&sgimabl_set_f->pitch_2006_c->pitch_2006_pid_s,
																																			&sgimabl_set_f->pitch_2006_c->pitch_2006_pid_p,
																																			0,
																																			sgimabl_set_f->pitch_2006_c->speed,
																																			-(ch1 - (sgimabl_set_f->pitch_2006_c->pitch2006_measure->actual_Position*360/8192/RESCUE_RATIO)),
																																			11000);
}

void rescue_set_remote(chassis_control_t *rescue_set_f, int16_t rescuespeed_set ,int16_t resclapspeed_set)
{
		rescue_set_f->rescue_motor_c->rescue_motor_output = motor_speed_control(&rescue_set_f->rescue_motor_c->rescue_motor_pid_s, rescuespeed_set,rescue_set_f->rescue_motor_c->rescue_motor_measure->speed,MAX_MOTOR_CAN_OUTPUT);
	
		rescue_set_f->rescue_clap_c->rescue_clap_output = motor_speed_control(&rescue_set_f->rescue_clap_c->rescue_clap_pid_s, resclapspeed_set,rescue_set_f->rescue_clap_c->rescue_clap_measure->speed,MAX_MOTOR_CAN_OUTPUT);
}




/**
  * @brief          ���̿���PID����
  * @param[in]      *chassis_pid_f���������ṹ��
  * @retval         none
  */
static void chassis_pid_calc(chassis_control_t *chassis_pid_f)
{
    uint8_t i = 0;

    if (*chassis_beh == CHASSIS_BATTERY) //��̨ģʽ������λ�û�����
    {
        /*���PIDλ�ñջ�����*/
        for (i = 0; i < 4; i++)
        {
            chassis_pid_f->chassis_motor[i].pid_output =
                motor_position_speed_control(&(chassis_pid_f->chassis_speed_pid[i]), &(chassis_pid_f->chassis_location_pid), 0, chassis_pid_f->chassis_motor[i].speed, 0, MAX_MOTOR_CAN_OUTPUT); //M3508_MAX_OUTPUT_CURRENT
        }
    }
    else // if (*chassis_beh == CHASSIS_FOLLOW)//����
    {
        /*���PID�ٶȱջ�����*/
        for (i = 0; i < 4; i++)
        {
            chassis_pid_f->chassis_motor[i].pid_output =
                motor_speed_control(&(chassis_pid_f->chassis_speed_pid[i]), chassis_pid_f->chassis_motor[i].speed_set, chassis_pid_f->chassis_motor[i].speed, MAX_MOTOR_CAN_OUTPUT); //M3508_MAX_OUTPUT_CURRENT
        }
    }
    //	else if (*chassis_beh == CHASSIS_NO_FOLLOW || *chassis_beh == CHASSIS_TWIST_WAIST || *chassis_beh == CHASSIS_ROTATION)//�����桢С���ݡ�Ť��
    //	{
    //		/*���PID�ٶȱջ�����*/
    //		for (i = 0; i < 4; i++)
    //		{
    //			chassis_pid_f->chassis_motor[i].pid_output = motor_speed_control(&(chassis_pid_f->chassis_rotate_pid), chassis_pid_f->chassis_motor[i].speed_set, chassis_pid_f->chassis_motor[i].speed, MAX_MOTOR_CAN_OUTPUT); //M3508_MAX_OUTPUT_CURRENT
    //		}
    //	}

#ifdef POWER_LIMIT //��������
//	chassis_power_control(chassis_pid_f);
//	chassis_power_limit_control(chassis_pid_f);
#endif
    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_pid_f->chassis_motor[i].give_current = (int16_t)(chassis_pid_f->chassis_motor[i].pid_output);
    }

#if CHASSIS_TEST_MODE
    chassis_jscope_print_curve();
#endif
}

/**
  * @brief          ���̼��ٶ�����б�º���
  * @param[in]      *ch0��
  * @param[in]      *ch1��
  * @param[in]      *ch2��
  * @retval         none
  */
static void chassis_accelerated_control(int16_t *ch0, int16_t *ch1, int16_t *ch2)
{
    static int16_t last_ch[3] = {0, 0, 0};
    int16_t temp[3];

    temp[0] = *ch0 - last_ch[0];
    temp[1] = *ch1 - last_ch[1];
    temp[2] = *ch2 - last_ch[2];

    if (chassis_control.chassis_RC->rc.s2 == RC_SW_UP) //ң��ģʽ
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;
        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;

        if (*chassis_beh == CHASSIS_TWIST_WAIST || *chassis_beh == CHASSIS_ROTATION) //Ť��ģʽ�²�����ת���ٶ�����
        {
            if (float_abs(temp[2]) > ROTATING_ACCELERAD)
                *ch2 = last_ch[2] + temp[2] / float_abs(temp[2]) * ROTATING_ACCELERAD;
        }
    }
    if (chassis_control.chassis_RC->rc.s2 == RC_SW_MID) //����ģʽ
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;
        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;

        if (*chassis_beh == CHASSIS_TWIST_WAIST) //Ť��ģʽ�²�����ת���ٶ�����
        {
            if (float_abs(temp[2]) > ROTATING_ACCELERAD)
                *ch2 = last_ch[2] + temp[2] / float_abs(temp[2]) * ROTATING_ACCELERAD;
        }
    }
    last_ch[0] = *ch0;
    last_ch[1] = *ch1;
    last_ch[2] = *ch2;
}

/**
  * @brief          ����ȫ���ر�
  * @param[in]      options:  0:������̿�����  1:��ֹ����   2:��ֹ���̺Ͳ�����
  * @retval         none
  * @attention  
  */
void chassis_task_off(u8 options)
{
    //������̿�����
    chassis_control.speed_x_set = 0;
    chassis_control.speed_y_set = 0;
    chassis_control.speed_z_set = 0;
    chassis_control.chassis_motor[0].speed_set = 0;
    chassis_control.chassis_motor[1].speed_set = 0;
    chassis_control.chassis_motor[2].speed_set = 0;
    chassis_control.chassis_motor[3].speed_set = 0;
    chassis_control.chassis_motor[0].give_current = 0;
    chassis_control.chassis_motor[1].give_current = 0;
    chassis_control.chassis_motor[2].give_current = 0;
    chassis_control.chassis_motor[3].give_current = 0;
    chassis_control.chassis_motor[0].pid_output = 0;
    chassis_control.chassis_motor[1].pid_output = 0;
    chassis_control.chassis_motor[2].pid_output = 0;
    chassis_control.chassis_motor[3].pid_output = 0;

    Chassis_Stop(&chassis_control);

    if (options)
    {
        can1_chassis_setmsg(0, 0, 0, 0);
        can1_chassis_gimbal_fire(0, 0, 0);
        system_state_set(INF_STOP);
    }
}

chassis_control_t *get_chassis_control_point(void)
{
    return &chassis_control;
}

#if CHASSIS_TEST_MODE
/* jscope��ӡ���� */
static void chassis_jscope_print_curve(void)
{
    JSCOPE_chassis_1 = chassis_control.chassis_motor[0].give_current;
    JSCOPE_chassis_2 = chassis_control.chassis_motor[1].give_current;
    JSCOPE_chassis_3 = chassis_control.chassis_motor[2].give_current;
    JSCOPE_chassis_4 = chassis_control.chassis_motor[3].give_current;
}
#endif
