/**
  ******************************************************************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务。
  ******************************************************************************
  */

#include "gimbal_task.h"
#include "gimbal_app.h"
#include "gimbal_behaviour.h"
#include "maths.h"
#include "rm_motor.h"
#include "filter.h"
#include "can_1_receive.h"
#include "can_2_receive.h"
#include "photoelectric.h"
#include "detect_task.h"
#include "rc.h"
#include "imu.h"
#include "iwdg.h"
#include "led.h"
#include "system_state.h"
#include "gpio_deal.h"
#include "vl53l0.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*--------------------变量-----------------------*/
//申明主云台变量
gimbal_control_t gimbal_control;
static const gimbal_behaviour_e *gimbal_beh;

//主云台pid设定位置
static float Gimbal_set_position = 0;
static __IO float Last_ch3 = 0;
static __IO float Now_ch3 = 0;
static uint16_t visual_shoot_speed = 0;
static uint16_t visual_red_blue = 0;

#if GIMBAL_TEST_MODE
int32_t pitch_real_jscope = 0;   //P轴打印实际曲线
int32_t pitch_set_jscope = 0;    //P轴打印设定曲线
int32_t yaw_real_jscope = 0;     //Y轴打印实际曲线
int32_t yaw_set_jscope = 0;      //Y轴打印设定曲线
int32_t sec_yaw_set_jscope = 0;  //副Y轴打印设定曲线
int32_t sec_yaw_real_jscope = 0; //副Y轴打印实际曲线
int32_t accel_up_jscope = 0;
int32_t accel_down_jscope = 0;
int JSCOPE_auto_pitch_angle = 0;
int JSCOPE_auto_yaw_angle = 0;
int JSCOPE_auto_kalman_pitch_angle = 0;
int JSCOPE_auto_kalman_yaw_angle = 0;
int JSCOPE_auto_pid_p = 0;
int JSCOPE_auto_pid_i = 0;
int JSCOPE_auto_pid_d = 0;
#endif

/*--------------------函数-----------------------*/
static void gimbal_task_control(gimbal_control_t *gimbal_task_control);
static void gimbal_remote_mode_choose(gimbal_control_t *fir_gimbal_choose);
static void gimbal_data_init(gimbal_control_t *gimbal_initialization);


static void function_control(gimbal_control_t *function_control_f);
static void function_data_update(gimbal_control_t *function_data_update_f);
static void function_pid_calc(gimbal_control_t *function_pid_f);
#if GIMBAL_TEST_MODE
/* jscope打印曲线 */
static void Gimba_jscope_print_curve(void);
#endif

gimbal_control_t *get_gimbal_control_point(void)
{
    return &gimbal_control;
}


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_task_stack;
#endif

/**
  * @brief  云台主任务
  * @param
  * @retval void
  */
//int temp_1, temp_2, Re_time;
void gimbal_task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    gimbal_data_init(&gimbal_control);

    GIMBAL_HEART = 0;

    while (1)
    {

//        temp_2 = xTaskGetTickCount(); //获取当前系统时间
//        Re_time = temp_2 - temp_1;
				VL53L0_Data_Deal();
		
				Get_Sensor_Val();

        gimbal_task_control(&gimbal_control); //云台工作
				
				function_control(&gimbal_control);

        /* 云台数据发送给底盘 */
        can2_gimbal_setmsg(gimbal_control.Gimbal_all_flag,                                        /*云台初始化标志*/
                           gimbal_control.yaw_c.photoelectric_zero,                               /*yaw轴光电值*/
                           gimbal_control.Gimbal_supply_flag,                                     /*补给状态标志位*/
                           (int16_t)((gimbal_control.yaw_c.chassis_gimbal_angel + 180.0f) * 100), /*底盘和云台yaw轴的差角*/
                           (uint8_t)*gimbal_beh,                                                  /*云台模式*/
                           (uint16_t)gimbal_control.pitch_c.pitch_motor_measure->actual_Position);

        /* 进行pitch轴3508电机和副云台的控制 */
        can1_gimbal_setmsg(gimbal_control.pitch_c.output, 0);
		can2_yaw_setmsg(gimbal_control.yaw_c.output);
			can2_sensor_setmsg(gimbal_control.sensor->Smooth_L, gimbal_control.sensor->Smooth_R);
			
			
			/* 上装的控制 */
				can1_function_setmsg(gimbal_control.uplift_c.output, gimbal_control.translation_c.output, 
															gimbal_control.telescoping_c.output, gimbal_control.clip_c.output);
				can1_flip_setmsg(gimbal_control.flip_c.output);

//        temp_1 = xTaskGetTickCount(); //获取当前系统时间

        vTaskDelay(GIMBAL_CONTROL_TIME_MS); //系统延时
		
#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_task_stack = uxTaskGetStackHighWaterMark(NULL); //堆栈的历史剩余最小值
#endif

    }
}

/**
  * @brief          云台数据初始化
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void gimbal_data_init(gimbal_control_t *Gimbal_data_init_f)
{
    /*--------------------获取指针--------------------*/
    {
        //获取遥控器指针(数据)
        Gimbal_data_init_f->gimbal_RC = get_remote_control_point();
        //获取副云台指针
        Gimbal_data_init_f->sec_gimbal_c.sec_gimbal_motor_measure = NULL;
        //获取云台指针
        Gimbal_data_init_f->yaw_c.yaw_motor_measure = get_yaw_gimbal_motor_measure_point();
        Gimbal_data_init_f->pitch_c.pitch_motor_measure = get_pitch_gimbal_motor_measure_point();
        //获取自瞄指针
        Gimbal_data_init_f->auto_c = Get_Auto_Control_Point();

        Gimbal_data_init_f->fire_c = get_fire_control_point();

        gimbal_beh = get_gimbal_behaviour_point();
        //获取陀螺仪指针
        Gimbal_data_init_f->imu_c = get_imu_control_point();
			
				Gimbal_data_init_f->vL53L0 = Return_VL53L0_t();
				Gimbal_data_init_f->sensor = Return_Sensor_t();
			
				//上装
				Gimbal_data_init_f->uplift_c.uplift_measure = get_uplift_measure_point();
				Gimbal_data_init_f->translation_c.translation_measure = get_translation_measure_point();
				Gimbal_data_init_f->telescoping_c.telescoping_measure = get_telescoping_measure_point();
				Gimbal_data_init_f->clip_c.clip_measure = get_clip_measure_point();
				Gimbal_data_init_f->flip_c.flip_measure = get_flip_measure_point();
    }

    /*--------------------检查遥控器数值是否正确--------------------*/
    rc_data_is_error();

    /*--------------------陀螺仪初始化--------------------*/
    {
        //imu_zero_correct();  //发送校准信号给陀螺仪模块
        Gimbal_data_init_f->yaw_c.last_angle = Gimbal_data_init_f->imu_c->yaw_angle; //更新初始角度
				Gimbal_data_init_f->flip_c.angle_init = (Gimbal_data_init_f->flip_c.flip_measure->actual_Position* 360/8192/FUNCTION_RATIO);
				Gimbal_data_init_f->flip_c.position_set = Gimbal_data_init_f->flip_c.angle_init;
    }

    /*--------------------滤波初始化--------------------*/
    {
        //pitch轴低通滤波
        first_order_filter_init(&Gimbal_data_init_f->pitch_c.LowFilt_Pitch_Data, Gimbal_Pitch_Fir_Ord_Low_Fil_Param);
        first_order_filter_init(&Gimbal_data_init_f->pitch_c.LowFilt_auto_pitch, 0.07642f); //0.0884f | 0.05642f
        //初始化P轴滑动滤波器
        sliding_mean_filter_init(&Gimbal_data_init_f->pitch_c.Slidmean_Pitch_Data);
        sliding_mean_filter_init(&Gimbal_data_init_f->pitch_c.Slidmean_auto_pitch);
    }

    /*--------------------PID初始化--------------------*/
    {
        //Pitch pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_p_pid, GIMBAL_P_PITCH_P, GIMBAL_P_PITCH_I, GIMBAL_P_PITCH_D, 2000, 0);
        Gimbal_data_init_f->pitch_c.pitch_p_pid.maximum = 180.0f;  //180.0f
        Gimbal_data_init_f->pitch_c.pitch_p_pid.minimum = -250.0f; //-250.0f
        Gimbal_data_init_f->pitch_c.pitch_p_pid.stepIn = 8.0f;
        Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmin = 0.5f;
        Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmax = 8.0f;
        //Gimbal_data_init_f->pitch_c.pitch_p_pid.deadband = 0.5f; //1.2
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_s_pid, GIMBAL_S_PITCH_P, GIMBAL_S_PITCH_I, GIMBAL_S_PITCH_D, 0, 0);

        //Yaw的pid参数初始化
        pid_init(&Gimbal_data_init_f->yaw_c.yaw_p_pid, GIMBAL_P_YAW_P, GIMBAL_P_YAW_I, GIMBAL_P_YAW_D, 0, 0);
        pid_init(&Gimbal_data_init_f->yaw_c.yaw_s_pid, GIMBAL_S_YAW_P, GIMBAL_S_YAW_I, GIMBAL_S_YAW_D, 0, 0);
        //yaw轴旋转pid参数初始化
        pid_init(&Gimbal_data_init_f->yaw_c.yaw_rotate_p_pid, GIMBAL_ROTATE_P_YAW_P, GIMBAL_ROTATE_P_YAW_I, GIMBAL_ROTATE_P_YAW_D, 0, 0);
        pid_init(&Gimbal_data_init_f->yaw_c.yaw_rotate_s_pid, GIMBAL_ROTATE_S_YAW_P, GIMBAL_ROTATE_S_YAW_I, GIMBAL_ROTATE_S_YAW_D, 0, 0);
			
				
				//上装
				pid_init(&Gimbal_data_init_f->uplift_c.uplift_p_pid, UPLIFT_P_P, UPLIFT_P_I, UPLIFT_P_D,0 ,0);
				pid_init(&Gimbal_data_init_f->uplift_c.uplift_s_pid, UPLIFT_S_P, UPLIFT_S_I, UPLIFT_S_D,0 ,0);
			
				pid_init(&Gimbal_data_init_f->translation_c.translation_p_pid, TRANSLATION_P_P, TRANSLATION_P_I, TRANSLATION_P_D,0 ,0);
				pid_init(&Gimbal_data_init_f->translation_c.translation_s_pid, TRANSLATION_S_P, TRANSLATION_S_I, TRANSLATION_S_D,0 ,0);
				
				pid_init(&Gimbal_data_init_f->telescoping_c.telescoping_p_pid, TELESCOP_P_P, TELESCOP_P_I, TELESCOP_P_D,0 ,0);
				pid_init(&Gimbal_data_init_f->telescoping_c.telescoping_s_pid, TELESCOP_S_P, TELESCOP_S_I, TELESCOP_S_D,0 ,0);
				
				pid_init(&Gimbal_data_init_f->clip_c.clip_p_pid, CLIP_P_P, CLIP_P_I, CLIP_P_D,0 ,0);
				pid_init(&Gimbal_data_init_f->clip_c.clip_s_pid, CLIP_S_P, CLIP_S_I, CLIP_S_D,0 ,0);
				
				pid_init(&Gimbal_data_init_f->flip_c.flip_p_pid, FLIP_P_P, FLIP_P_I, FLIP_P_D,0 ,0);
				pid_init(&Gimbal_data_init_f->flip_c.flip_s_pid, FLIP_S_P, FLIP_S_I, FLIP_S_D,0 ,0);
			
//////////短焦
		//gimbal_change_pid(3);
//////////工业
		gimbal_change_pid(2);
	}
    /*--------------------设置开机状态--------------------*/
    gimbal_task_off(1);
		Gimbal_data_init_f->uplift_c.complete_state = DisFinish;
		Gimbal_data_init_f->translation_c.complete_state = DisFinish;
		Gimbal_data_init_f->telescoping_c.complete_state = DisFinish;
		Gimbal_data_init_f->clip_c.complete_state = DisFinish;
		Gimbal_data_init_f->flip_c.complete_state = DisFinish;
		Gimbal_data_init_f->robo_state = 0;
}

/**
  * @brief          云台状态控制
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void gimbal_task_control(gimbal_control_t *gimbal_task_control_f)
{
    correct_yaw_zero(gimbal_task_control_f); //yaw轴码盘修正

    /* 实时监测Yaw校准光电  (到中值返回1) */
    gimbal_task_control_f->yaw_c.photoelectric_zero = yaw_zero_value();
	
		gimbal_task_control_f->sensor->Smooth_L = Return_Smooth_L_Val();
		gimbal_task_control_f->sensor->Smooth_R = Return_Smooth_R_Val();
	
		gimbal_task_control_f->vL53L0->InitDistance = return_vl53l0_initdistance();
		gimbal_task_control_f->vL53L0->distance = return_vl53l0_distance();
    if (gimbal_task_control_f->Gimbal_all_flag == 1)
    {
        gyro_usart_iwdg();
    } //给陀螺仪喂狗粮 在右边为下的时候不会进入这里，所以陀螺仪数据初始化，angle为0
	imu_data_reception(); //接收陀螺仪数值
    //imu_zero_correct();

    gimbal_task_control_f->yaw_c.chassis_gimbal_angel = -get_yaw_different_angle(gimbal_task_control_f->yaw_c.yaw_motor_measure, YAW_RATIO);

    //小电脑通讯
    visual_data_reception();  //通过串口三接收并保存MiniPC发来的数据
	visual_shoot_speed = re_can2_shooter_heat0_speed_limit();
	//visual_red_blue = 
	if (visual_shoot_speed >= 15 && visual_shoot_speed <= 30)
	{
		visual_send_data(1, 0, visual_shoot_speed - 2);
	}
	else
	{
		visual_send_data(1, 0, 13);
	}

    //云台模式选择
    gimbal_remote_mode_choose(gimbal_task_control_f);
}

/**
  * @brief          云台行为选择
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void gimbal_remote_mode_choose(gimbal_control_t *fir_gimbal_choose)
{
    gimbal_behaviour_mode_set(fir_gimbal_choose);
}

/**
  * @brief          云台正常控制
  * @param[in]      none
  * @retval         none
  * @attention
  */
void gimbal_manual_work(gimbal_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3)
{
    gimbal_working->yaw_c.angle = gimbal_working->imu_c->yaw_angle - gimbal_working->yaw_c.last_angle; //获取云台正常工作时的陀螺仪Y轴角度

    /* 主云台控制 前一个变量为遥控器控制，后一个变量为控制地面绝对角度 */
    Gimbal_set_position = loop_fp32_constrain((-gimbal_ch2 - (YAW_ANGLE_FLAG * gimbal_working->yaw_c.angle)), -180.0f, 180.0f); //云台设定位置循环限幅

    /* P轴加速度 速度的PID微分 */
    //	gimbal_working->pitch_c.accel_down = gimbal_working->pitch_c.pitch_down_s_pid.Derror[0] * 100.0f;
    //	gimbal_working->pitch_c.accel_up = gimbal_working->pitch_c.pitch_up_s_pid.Derror[0] * 100.0f;

    //	Data_Accelerated_Control();

    /* P轴滤波 */
    gimbal_working->pitch_c.output = sliding_mean_filter(&gimbal_working->pitch_c.Slidmean_Pitch_Data, gimbal_working->pitch_c.output, 55); //均值滑窗滤波（有滞后）
    gimbal_working->pitch_c.output = first_order_filter(&gimbal_working->pitch_c.LowFilt_Pitch_Data, gimbal_working->pitch_c.output);       //一阶低通滤波

//    Now_ch3 = gimbal_ch3;
//    Last_ch3 = Now_ch3;
//    if (Now_ch3 - Last_ch3 > 0.0f)

    /* Pitch位置控制 */
    gimbal_working->pitch_c.output = motor_position_Stepping(&gimbal_working->pitch_c.pitch_s_pid,
                                                             &gimbal_working->pitch_c.pitch_p_pid,
                                                             0,                                                                                             /*真实位置*/
                                                             gimbal_working->imu_c->Gyro_X,                                                                 /*真实速度*/
                                                             -((gimbal_ch3) - (gimbal_working->pitch_c.pitch_motor_measure->actual_Position * 360 / 1024)), /*设定位置*/
                                                             PITCH_OUTPUT_LIMIT);                                                                           /*输出限制*/

    /* Yaw位置控制 以步兵的方向来讲 yaw轴左正右负 pitch轴上负下正 */
    gimbal_working->yaw_c.output = motor_position_speed_control(&gimbal_working->yaw_c.yaw_s_pid,
                                                                &gimbal_working->yaw_c.yaw_p_pid,
                                                                0,                                              /*真实位置*/
                                                                gimbal_working->yaw_c.yaw_motor_measure->speed, /*真实速度*/
                                                                Gimbal_set_position,                            /*设定位置*/
                                                                YAW_OUTPUT_LIMIT);                              /*输出限制*/

#if GIMBAL_TEST_MODE
    /*打印曲线*/
    Gimba_jscope_print_curve();
#endif
}

/**
  * @brief          自瞄模式
  * @param[in]      none
  * @retval         none
  * @attention
  */
void gimbal_automatic_work(gimbal_control_t *gimbal_auto_work_f)
{
    // NO_4 修改增加
    if (gimbal_auto_work_f->auto_c->auto_lost_data_flag == 1) //丢失数据
    {
		//TODO 测试这里是分开pid结构体，尝试同一结构体改参数
		if (gimbal_auto_work_f->auto_c->auto_pitch_angle <= 0.0f)
		{
			gimbal_change_pid(0);//0: 上升
		}
		else
		{
			gimbal_change_pid(1);//0: 下降
		}
		
		gimbal_auto_work_f->pitch_c.output = 
			-motor_position_Stepping(&gimbal_auto_work_f->pitch_c.pitch_auto_s_pid,
			                         &gimbal_auto_work_f->pitch_c.pitch_auto_p_pid,
			                         gimbal_auto_work_f->pitch_c.pitch_motor_measure->actual_Position,   /*真实位置*/
			                         gimbal_auto_work_f->imu_c->Gyro_X,                                  /*真实速度*/
			                         (gimbal_auto_work_f->pitch_c.Auto_record_location),                 /*设定位置*/
			                         PITCH_OUTPUT_LIMIT); 

        /*输出量闭环处理*/
        gimbal_auto_work_f->yaw_c.output = motor_position_speed_control(&gimbal_auto_work_f->yaw_c.yaw_auto_s_pid,
                                                                        &gimbal_auto_work_f->yaw_c.yaw_auto_p_pid,
                                                                        0,
                                                                        gimbal_auto_work_f->yaw_c.yaw_motor_measure->speed, /*真实速度*/
                                                                        loop_fp32_constrain(-(gimbal_auto_work_f->imu_c->yaw_angle - gimbal_auto_work_f->yaw_c.last_angle), -180.0f, 180.0f),
                                                                        YAW_OUTPUT_LIMIT);
    }
    else
    {
		//TODO 测试这里是分开pid结构体，尝试同一结构体改参数
		if (gimbal_auto_work_f->auto_c->auto_pitch_angle <= 0.0f)
		{
			gimbal_change_pid(0);//0: 上升
		}
		else
		{
			gimbal_change_pid(1);//0: 下降
		}
		
		//motor_position_Stepping
        gimbal_auto_work_f->pitch_c.output = motor_position_Stepping(&gimbal_auto_work_f->pitch_c.pitch_auto_s_pid,
                                                                     &gimbal_auto_work_f->pitch_c.pitch_auto_p_pid,
                                                                     0,
                                                                     gimbal_auto_work_f->imu_c->Gyro_X,
                                                                     (gimbal_auto_work_f->auto_c->pitch_control_data),
                                                                     PITCH_OUTPUT_LIMIT);

        /*输出量闭环处理*/
        gimbal_auto_work_f->yaw_c.output = motor_position_Stepping(&gimbal_auto_work_f->yaw_c.yaw_auto_s_pid,
                                                                   &gimbal_auto_work_f->yaw_c.yaw_auto_p_pid,
                                                                   0,
                                                                   gimbal_auto_work_f->yaw_c.yaw_motor_measure->speed,
                                                                   (-gimbal_auto_work_f->auto_c->yaw_control_data),
                                                                   YAW_OUTPUT_LIMIT); //Yaw位置控制
    }

//	gimbal_auto_work_f->pitch_c.output = Sliding_Mean_Filter(&gimbal_auto_work_f->pitch_c.Slidmean_Pitch_Data, gimbal_auto_work_f->pitch_c.output, 55); //均值滑窗滤波（有滞后）
//	gimbal_auto_work_f->pitch_c.output = first_order_filter(&gimbal_auto_work_f->pitch_c.LowFilt_Pitch_Data, gimbal_auto_work_f->pitch_c.output);       //一阶低通滤波
//
//	/* Pitch位置控制 */
//	gimbal_auto_work_f->pitch_c.output = Motor_Position_Speed_Control(&gimbal_auto_work_f->pitch_c.pitch_down_s_pid,
//																  &gimbal_auto_work_f->pitch_c.pitch_down_p_pid,
//																  0,                                                            /*真实位置*/
//																  IMU_t.Gyro_X,                                                 /*真实速度*/
//																  -((0) - (gimbal_auto_work_f->pitch_c.pitch_motor_measure->actual_Position * 360 / 1024)), /*设定位置*/
//																  PITCH_OUTPUT_LIMIT);                                          /*输出限制*/		/* P轴滤波 */

#if GIMBAL_TEST_MODE
    Gimba_jscope_print_curve();
#endif
}

/**
  * @brief          云台全部关闭
  * @param[in]      options:  0:清除云台控制量  1:禁止云台   2:禁止云台yaw轴和pitch轴
  * @retval         none
  * @attention      0: 状态接切换
  *                 1: 初次进入状态
  *                 2: 清除初始化标志位
  */
void gimbal_task_off(u8 options)
{
    //控制量
    gimbal_control.pitch_c.output = 0;
    gimbal_control.yaw_c.output = 0;
    gimbal_control.sec_gimbal_c.output = 0;

    //自瞄控制量
    gimbal_control.auto_c->auto_pitch_angle = 0.0f;
    gimbal_control.auto_c->auto_yaw_angle = 0.0f;
    gimbal_control.auto_c->pitch_control_data = 0.0f;
    gimbal_control.auto_c->yaw_control_data = 0.0f;
    gimbal_control.pitch_c.Auto_record_location = 0.0f;

    Gimbal_set_position = 0;
    gimbal_control.yaw_c.last_angle = gimbal_control.imu_c->yaw_angle; //更新初始角度

    if (options == 0)
    {
        gimbal_ch_retain(&gimbal_control);
    }
    else if (options == 1)
    {
        gimbal_ch_set0(); //TODO
    }
    else if (options == 2)
    {
        gimbal_control.Gimbal_all_flag = 0;
        system_state_set(INF_STOP);
        gimbal_ch_set0();         //TODO
        can1_gimbal_setmsg(0, 0); //pitch轴
        can2_yaw_setmsg(0);       //yaw轴
    }
}

void gimbal_change_pid(int x)
{
    if (x == 0) //自瞄：pitch轴上升
    {
		//自瞄工业：Pitch的pid参数初始化
		gimbal_control.pitch_c.pitch_auto_p_pid.Kp = 18.0f;
		gimbal_control.pitch_c.pitch_auto_p_pid.Ki = 6.0f;
		gimbal_control.pitch_c.pitch_auto_p_pid.Kd = 35.0f;
		gimbal_control.pitch_c.pitch_auto_s_pid.Kp = 8.0f;
		gimbal_control.pitch_c.pitch_auto_s_pid.Ki = 0.1f;
		gimbal_control.pitch_c.pitch_auto_s_pid.Kd = 1.0f;
    }
    else if (x == 1) //自瞄：pitch轴下降
    {
		//自瞄工业：Pitch的pid参数初始化
		gimbal_control.pitch_c.pitch_auto_p_pid.Kp = 0.25f;
		gimbal_control.pitch_c.pitch_auto_p_pid.Ki = 2.0f;
		gimbal_control.pitch_c.pitch_auto_p_pid.Kd = 40.0f;
		gimbal_control.pitch_c.pitch_auto_s_pid.Kp = 4.0f;
		gimbal_control.pitch_c.pitch_auto_s_pid.Ki = 0.1f;
		gimbal_control.pitch_c.pitch_auto_s_pid.Kd = 1.0f;
    }
    else if (x == 2) //工业自瞄
    {
		pid_clear(&gimbal_control.yaw_c.yaw_auto_p_pid);
		pid_clear(&gimbal_control.yaw_c.yaw_auto_s_pid);
		pid_clear(&gimbal_control.pitch_c.pitch_auto_p_pid);
		pid_clear(&gimbal_control.pitch_c.pitch_auto_s_pid);
		
		//自瞄工业：Yaw的pid参数初始化
        pid_init(&gimbal_control.yaw_c.yaw_auto_p_pid, GIMBAL_AUTO_INDUSTRY_P_YAW_P, GIMBAL_AUTO_INDUSTRY_P_YAW_I, GIMBAL_AUTO_INDUSTRY_P_YAW_D, 0, 0);
        gimbal_control.yaw_c.yaw_auto_p_pid.maximum = 500.0f;
        gimbal_control.yaw_c.yaw_auto_p_pid.minimum = -500.0f;
        gimbal_control.yaw_c.yaw_auto_p_pid.stepIn = 10.0f;
        gimbal_control.yaw_c.yaw_auto_p_pid.errorabsmin = 1;
        gimbal_control.yaw_c.yaw_auto_p_pid.errorabsmax = 6;
		gimbal_control.yaw_c.yaw_auto_p_pid.max_iout = 1000;
        pid_init(&gimbal_control.yaw_c.yaw_auto_s_pid, GIMBAL_AUTO_INDUSTRY_S_YAW_P, GIMBAL_AUTO_INDUSTRY_S_YAW_I, GIMBAL_AUTO_INDUSTRY_S_YAW_D, 0, 0);
		
		//自瞄工业：Pitch的pid参数初始化
        pid_init(&gimbal_control.pitch_c.pitch_auto_p_pid, GIMBAL_AUTO_INDUSTRY_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_P_PITCH_D, 0, 0);
//        gimbal_control.pitch_c.pitch_auto_p_pid.maximum = 180.0f;
//        gimbal_control.pitch_c.pitch_auto_p_pid.minimum = -250.0f;
        gimbal_control.pitch_c.pitch_auto_p_pid.stepIn = (4.0f * 5.0f);
        gimbal_control.pitch_c.pitch_auto_p_pid.errorabsmin = (0.2f * 5.0f);
        gimbal_control.pitch_c.pitch_auto_p_pid.errorabsmax = (4.0f * 5.0f);
		gimbal_control.pitch_c.pitch_auto_p_pid.max_iout = 1000;
        pid_init(&gimbal_control.pitch_c.pitch_auto_s_pid, GIMBAL_AUTO_INDUSTRY_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_S_PITCH_D, 0, 0);
    }
    else if (x == 3) //短焦
    {
		pid_clear(&gimbal_control.yaw_c.yaw_auto_p_pid);
		pid_clear(&gimbal_control.yaw_c.yaw_auto_s_pid);
		pid_clear(&gimbal_control.pitch_c.pitch_auto_p_pid);
		pid_clear(&gimbal_control.pitch_c.pitch_auto_s_pid);
		
        //自瞄短焦：Yaw的pid参数初始化
        pid_init(&gimbal_control.yaw_c.yaw_auto_p_pid, GIMBAL_AUTO_SHORT_P_YAW_P, GIMBAL_AUTO_SHORT_P_YAW_I, GIMBAL_AUTO_SHORT_P_YAW_D, 1000, 0);
        gimbal_control.yaw_c.yaw_auto_p_pid.maximum = 500.0f;
        gimbal_control.yaw_c.yaw_auto_p_pid.minimum = -500.0f;
        gimbal_control.yaw_c.yaw_auto_p_pid.stepIn = 10.0f;
        gimbal_control.yaw_c.yaw_auto_p_pid.errorabsmin = 1;
        gimbal_control.yaw_c.yaw_auto_p_pid.errorabsmax = 3;
		gimbal_control.yaw_c.yaw_auto_p_pid.max_iout = 1000;
        pid_init(&gimbal_control.yaw_c.yaw_auto_s_pid, GIMBAL_AUTO_SHORT_S_YAW_P, GIMBAL_AUTO_SHORT_S_YAW_I, GIMBAL_AUTO_SHORT_S_YAW_D, 0, 0);
        //自瞄短焦：Pitch的pid参数初始化
        pid_init(&gimbal_control.pitch_c.pitch_auto_p_pid, GIMBAL_AUTO_SHORT_P_PITCH_P, GIMBAL_AUTO_SHORT_P_PITCH_I, GIMBAL_AUTO_SHORT_P_PITCH_D, 2000, 0);
        gimbal_control.pitch_c.pitch_auto_p_pid.maximum = 180.0f;
        gimbal_control.pitch_c.pitch_auto_p_pid.minimum = -250.0f;
        gimbal_control.pitch_c.pitch_auto_p_pid.stepIn = 8.0f;
        gimbal_control.pitch_c.pitch_auto_p_pid.errorabsmin = (0.5f * 5.0f);
        gimbal_control.pitch_c.pitch_auto_p_pid.errorabsmax = (8.0f * 5.0f);
		gimbal_control.pitch_c.pitch_auto_p_pid.max_iout = 1000;
        pid_init(&gimbal_control.pitch_c.pitch_auto_s_pid, GIMBAL_AUTO_SHORT_S_PITCH_P, GIMBAL_AUTO_SHORT_S_PITCH_I, GIMBAL_AUTO_SHORT_S_PITCH_D, 0, 0);
    }
}

#if GIMBAL_TEST_MODE
/* jscope打印曲线 */
static void Gimba_jscope_print_curve(void)
{
    pitch_real_jscope = gimbal_control.pitch_c.pitch_motor_measure->speed; //P轴打印实际曲线
    pitch_set_jscope = gimbal_control.pitch_c.output;                      //P轴打印设定曲线

    yaw_real_jscope = gimbal_control.yaw_c.yaw_motor_measure->speed; //Y轴打印实际曲线
    yaw_set_jscope = gimbal_control.yaw_c.output;                    //Y轴打印设定曲线

    sec_yaw_set_jscope = 0;  //副Y轴打印设定曲线
    sec_yaw_real_jscope = 0; //副Y轴打印实际曲线

    accel_up_jscope = gimbal_control.pitch_c.accel_up;
    accel_down_jscope = gimbal_control.pitch_c.accel_down;

    JSCOPE_auto_pitch_angle = gimbal_control.auto_c->auto_pitch_angle * 100;
    JSCOPE_auto_yaw_angle = gimbal_control.auto_c->auto_yaw_angle * 100;
	JSCOPE_auto_kalman_pitch_angle = gimbal_control.auto_c->auto_kalman_pitch_angle * 100;
    JSCOPE_auto_kalman_yaw_angle = gimbal_control.auto_c->auto_kalman_yaw_angle * 100;
	
	JSCOPE_auto_pid_p = gimbal_control.pitch_c.pitch_auto_p_pid.Pout;
    JSCOPE_auto_pid_i = gimbal_control.pitch_c.pitch_auto_p_pid.Dout;
	JSCOPE_auto_pid_d = gimbal_control.pitch_c.pitch_auto_p_pid.Iout;
}
#endif


/*==================================工程云台========================================*/
static void function_control(gimbal_control_t *function_control_f)
{
		//底盘数据更新
		function_data_update(function_control_f);	
	
		//遥控器模式状态设置
		gimbal_remote_mode_choose(function_control_f);
	
		//底盘控制PID计算
		function_pid_calc(function_control_f);
}


static void function_data_update(gimbal_control_t *function_data_update_f)
{
		function_data_update_f->uplift_c.speed = function_data_update_f->uplift_c.uplift_measure->speed;
	
		function_data_update_f->translation_c.speed = function_data_update_f->translation_c.translation_measure->speed;
	
		function_data_update_f->telescoping_c.speed = function_data_update_f->telescoping_c.telescoping_measure->speed;
	
		function_data_update_f->clip_c.speed = function_data_update_f->clip_c.clip_measure->speed;
	
		function_data_update_f->flip_c.speed = function_data_update_f->flip_c.flip_measure->speed;
}

static void function_pid_calc(gimbal_control_t *function_pid_f)
{
		//抬升
		if (function_pid_f->uplift_c.complete_state == DisFinish) //未完成跑速度环
		{
				function_pid_f->uplift_c.output = motor_speed_control(&function_pid_f->uplift_c.uplift_s_pid, function_pid_f->uplift_c.speed_set, function_pid_f->uplift_c.uplift_measure->speed,MAX_MOTOR_CAN_OUTPUT);
		}
		else if (function_pid_f->uplift_c.complete_state == Finish) //否则跑位置环
		{
				function_pid_f->uplift_c.output = motor_position_speed_control(&function_pid_f->uplift_c.uplift_s_pid, &function_pid_f->uplift_c.uplift_p_pid,
																		function_pid_f->uplift_c.last_angle, function_pid_f->uplift_c.speed,
																		(function_pid_f->uplift_c.uplift_measure->actual_Position*360/8192/LIFT_RATIO),11000);
		}
		else
		{
				function_pid_f->uplift_c.output = 0;
		}
	
		
		
		//平移
		if (function_pid_f->translation_c.complete_state == DisFinish) //未完成跑速度环
		{
				function_pid_f->translation_c.output = motor_speed_control(&function_pid_f->translation_c.translation_s_pid, function_pid_f->translation_c.speed_set, function_pid_f->translation_c.translation_measure->speed,MAX_MOTOR_CAN_OUTPUT);
		}
		else if (function_pid_f->translation_c.complete_state == Finish) //否则跑位置环
		{
				function_pid_f->translation_c.output = motor_position_speed_control(&function_pid_f->translation_c.translation_s_pid, &function_pid_f->translation_c.translation_p_pid,
																		function_pid_f->translation_c.last_angle, function_pid_f->translation_c.speed,
																		(function_pid_f->translation_c.translation_measure->actual_Position*360/8192/RESCUE_RATIO),11000);
		}
		else
		{
				function_pid_f->translation_c.output = 0;
		}

		
		
		//伸缩
		if (function_pid_f->telescoping_c.complete_state == DisFinish) //未完成跑速度环
		{
				function_pid_f->telescoping_c.output = motor_speed_control(&function_pid_f->telescoping_c.telescoping_s_pid, function_pid_f->translation_c.speed_set, function_pid_f->translation_c.translation_measure->speed,MAX_MOTOR_CAN_OUTPUT);
		}
		else if (function_pid_f->telescoping_c.complete_state == Finish) //否则跑位置环
		{
				function_pid_f->telescoping_c.output = motor_position_speed_control(&function_pid_f->telescoping_c.telescoping_s_pid, &function_pid_f->telescoping_c.telescoping_p_pid,
																		function_pid_f->telescoping_c.last_angle, function_pid_f->telescoping_c.speed,
																		(function_pid_f->telescoping_c.telescoping_measure->actual_Position*360/8192/FUNCTION_RATIO),11000);
		}
		else
		{
				function_pid_f->telescoping_c.output = 0;
		}
		
		//夹子
		if (function_pid_f->clip_c.complete_state == DisFinish) //未完成跑速度环
		{
				function_pid_f->clip_c.output = motor_speed_control(&function_pid_f->clip_c.clip_s_pid, function_pid_f->clip_c.speed_set, function_pid_f->clip_c.clip_measure->speed,MAX_MOTOR_CAN_OUTPUT);
		}
		else if (function_pid_f->clip_c.complete_state == Finish) //否则跑位置环
		{
				function_pid_f->clip_c.output = motor_position_speed_control(&function_pid_f->clip_c.clip_s_pid, &function_pid_f->clip_c.clip_p_pid,
																		function_pid_f->clip_c.last_angle, function_pid_f->clip_c.speed,
																		(function_pid_f->clip_c.clip_measure->actual_Position*360/8192/FUNCTION_RATIO),11000);
		}
		else
		{
				function_pid_f->clip_c.output = 0;
		}
		
		//翻转
		function_pid_f->flip_c.output = motor_position_speed_control(&function_pid_f->flip_c.flip_s_pid, &function_pid_f->flip_c.flip_p_pid,
																function_pid_f->flip_c.position_set, function_pid_f->clip_c.speed,
																(function_pid_f->flip_c.flip_measure->actual_Position*360/8192/FUNCTION_RATIO),11000);

}



































