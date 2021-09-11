#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "maths.h"
#include "rm_motor.h"
#include "filter.h"
#include "vl53l0.h"
#include "can_1_receive.h"
#include "can_2_receive.h"
#include "imu.h"
#include "photoelectric.h"
#include "detect_task.h"
#include "rc.h"
#include "init.h"
#include "visual.h"
#include "shoot_task.h"
#include "parameter.h"
#include "system_state.h"

//键盘鼠标选择
static void mk_data_process(gimbal_control_t *MK_Data_Process_f);
//云台模式选择
static void gimbal_mode_choose(gimbal_control_t *fir_gimbal_mode_choose_f);
//云台手动模式
static void gimbal_manual(gimbal_control_t *gimbal_manual_f);
//云台自动瞄准
static void gimbal_auto(gimbal_control_t *gimbal_auto_f);

static void uplift_behav(gimbal_control_t *uplift_motor, VL53L0_t *vl53l0, float dis, u8 dire);
static void translation_behav(gimbal_control_t *translation_motor, u8 dire);
static void telescoping_behav(gimbal_control_t *telescoping_motor, u8 dire);
static void clip_behav(gimbal_control_t *clip_motor, u8 dire);
static void flip_behav(gimbal_control_t *filp_motor, float exp, float limit, u8 dire);

static void grasp_init(gimbal_control_t *grasp_motor_init);
static void grasp_reset(gimbal_control_t *grasp_motor_reset);
static void grasp_first(gimbal_control_t *grasp_motor_first);
static void grasp_second(gimbal_control_t *grasp_motor_second);
static void grasp_third(gimbal_control_t *grasp_motor_third);
static void flap_reset(gimbal_control_t *flap_motor_reset);

//static
gimbal_behaviour_e gimbal_behaviour;
gimbal_behaviour_e gimbal_last_behaviour;

gimbal_behaviour_e grasp_behaviour;
float Gimbal_ch2 = 0.0f, Gimbal_ch3 = 0.0f; //云台电机受控量
uint16_t shoot_speed_record = 0;


/**
  * @brief          初始化调用
  * @param[in]      none
  * @retval         none
  * @attention
  */
void gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f)
{
    //!模式选择
    gimbal_mode_choose(fir_gimbal_behaviour_f);

    // NO_4 修改增加
    if (gimbal_last_behaviour != gimbal_behaviour)
    {
        gimbal_task_off(0);
        gimbal_last_behaviour = gimbal_behaviour;
    }

    if ((system_state_return() != INF_STOP))
    {
        if (system_state_return() == INF_MK) //TODO 状态切换
        {
            //*鼠标控制云台 手动模式manual
//            if (gimbal_behaviour == GIMBAL_MANUAL)
//            {
//                gimbal_manual(fir_gimbal_behaviour_f);
//            }
//            //若为自瞄模式automatic
//            else if (gimbal_behaviour == GIMBAL_AUTOATTACK)
//            {
//                gimbal_auto(fir_gimbal_behaviour_f);
//            }
//						
						if (grasp_behaviour == GRASP_INIT)
						{
								grasp_init(fir_gimbal_behaviour_f);
						}
						if (grasp_behaviour == GRASP_FIRST)
						{
								grasp_reset(fir_gimbal_behaviour_f);
						}
						if (grasp_behaviour == GRASP_RESET)
						{
								grasp_first(fir_gimbal_behaviour_f);
						}
						if (grasp_behaviour == GRASP_SECOND)
						{
								grasp_second(fir_gimbal_behaviour_f);
						}
						if (grasp_behaviour == GRASP_THIRD)
						{
								grasp_third(fir_gimbal_behaviour_f);
						}
						if (grasp_behaviour == FLIP_RESET)
						{
								flap_reset(fir_gimbal_behaviour_f);
						}
						
        }
        else
        {
            //若为手动模式manual
            if (gimbal_behaviour == GIMBAL_MANUAL)
            {
                gimbal_manual(fir_gimbal_behaviour_f);
            }
            //若为自瞄模式automatic
            else if (gimbal_behaviour == GIMBAL_AUTOATTACK)
            {
                gimbal_auto(fir_gimbal_behaviour_f);
            }
        }
    }
    else
    {
        remote_reload();    //摇杆量清零
        gimbal_task_off(2); //停止   TODO
    }
}

static void gimbal_mode_choose(gimbal_control_t *fir_gimbal_mode_choose_f) //TODO 状态切换注释了
{
    //右开关打上，遥控控制
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW_UP)
    {
        if (fir_gimbal_mode_choose_f->Gimbal_all_flag != 1)
        {
            calibrate_task_remove_hang(); //解除初始化校准任务
        }
        else if (fir_gimbal_mode_choose_f->Gimbal_all_flag == 1)
        {
            system_state_set(INF_RC);

            gimbal_behaviour = GIMBAL_MANUAL; //状态设置为手动
        }
    }
    //右开关打中，键盘控制
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW_MID)
    {
//        if (fir_gimbal_mode_choose_f->Gimbal_all_flag != 1)
//        {
//            calibrate_task_remove_hang(); //解除初始化校准任务
//        }

            if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW_UP)
            {
                system_state_set(INF_RC);

                gimbal_behaviour = GIMBAL_AUTOBUFF; //打符模式
            }
            else if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW_MID)
            {
                if (system_state_return() != INF_MK)
                {
                    gimbal_behaviour = GIMBAL_MANUAL; //状态设置为手动
                    gimbal_last_behaviour = GIMBAL_MANUAL;
                    gimbal_task_off(1);
                }
                system_state_set(INF_MK);
                mk_data_process(fir_gimbal_mode_choose_f); //!键盘模式
            }
            else if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s1 == RC_SW_DOWN)
            {
                system_state_set(INF_RC);
                gimbal_behaviour = GIMBAL_AUTOATTACK; //自瞄模式
            }
        
    }
    //右开关打下，停止工作
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW_DOWN)
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
        system_state_set(INF_STOP);
        fir_gimbal_mode_choose_f->Gimbal_all_flag = 0; //初始化标志位清零
    }
    //出现严重错误
    if (fir_gimbal_mode_choose_f->gimbal_RC->rc.s2 == RC_SW_ERROR)
    {
    }
}

/**
  * @brief          键盘鼠标选择
  * @param[in]      *MK_Data_Process_f：底盘主结构体
  * @retval         none
  * @attention
  */
void mk_data_process(gimbal_control_t *MK_Data_Process_f)
{
    /* 鼠标控制 */
    {
        //**鼠标右键(按住)   自瞄
        if (MK_Data_Process_f->gimbal_RC->mouse.press_r == 1)
		{ 
            gimbal_behaviour = GIMBAL_AUTOATTACK;  //自瞄模式
		} 
		else
		{
			gimbal_behaviour = GIMBAL_MANUAL; //手动模式
		}
//        //**鼠标左键(单击或长按)   发射
//		if (MK_Data_Process_f->gimbal_RC->mouse.press_l == 1)
//        {
//            
//        }
		
		
    //**Q
    if (MK_Data_Process_f->gimbal_RC->kb.bit.Q)
    {
				grasp_behaviour = GRASP_INIT;
    }

    //**E
    if (MK_Data_Process_f->gimbal_RC->kb.bit.E)
    {
				grasp_behaviour = GRASP_FIRST;
    }

    //**R
    if (MK_Data_Process_f->gimbal_RC->kb.bit.R)
    {
				grasp_behaviour = GRASP_RESET;
    }
    else if (MK_Data_Process_f->gimbal_RC->kb.bit.R && MK_Data_Process_f->gimbal_RC->kb.bit.CTRL)
    {
				grasp_behaviour = FLIP_RESET;
    }
		
    //**V
    if (MK_Data_Process_f->gimbal_RC->kb.bit.V)
    {
				grasp_behaviour = GRASP_SECOND;
    }

    //**B
    if (MK_Data_Process_f->gimbal_RC->kb.bit.B)
    {
				grasp_behaviour = GRASP_THIRD;
    }
		
    }
}

/**
  * @brief          云台手动模式
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void gimbal_manual(gimbal_control_t *gimbal_manual_f)
{
    if (system_state_return() == INF_MK)
    {
        Gimbal_ch2 += (gimbal_manual_f->gimbal_RC->mouse.x * MOUSE_YAW_SPEED * 0.5f);   //Y轴位置环量累加
        Gimbal_ch2 = loop_fp32_constrain(Gimbal_ch2, -180.0f, 180.0f);                  //循环限幅
        Gimbal_ch3 -= (gimbal_manual_f->gimbal_RC->mouse.y * MOUSE_PITCH_SPEED * 0.4f); //P轴位置环量累加

        //pitch角度限制   -22 ~ 38
        Gimbal_ch3 = float_limit(Gimbal_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);
    }
    else
    {
        Gimbal_ch2 += (gimbal_manual_f->gimbal_RC->rc.ch[2]) * RC_YAW_SPEED * 0.5f;   //Y轴位置环量累加   RC_YAW_SPEED
        Gimbal_ch2 = loop_fp32_constrain(Gimbal_ch2, -180.0f, 180.0f);                //循环限幅
        Gimbal_ch3 += (gimbal_manual_f->gimbal_RC->rc.ch[3]) * RC_PITCH_SPEED * 0.3f; //P轴位置环量累加  RC_PITCH_SPEED

        //pitch角度限制   -22 ~ 38
        Gimbal_ch3 = float_limit(Gimbal_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);
    }

    gimbal_manual_work(gimbal_manual_f, Gimbal_ch2, Gimbal_ch3);
}

/**
  * @brief          云台自动瞄准
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void gimbal_auto(gimbal_control_t *gimbal_auto_f)
{
    //Change_camera(1); //自瞄模式下切换摄像头参数为短焦相机

    /*预处理*/
    gimbal_auto_f->yaw_c.angle = gimbal_auto_f->imu_c->yaw_angle - gimbal_auto_f->yaw_c.last_angle; //获取云台自动打击时的陀螺仪Y轴角度
                                                                                                    //	Kalman_Filter_Calc(&kalman_Filter, vision_auto_data.auto_yaw_angle+Gimbal_yaw_angle, vision_auto_data.auto_pitch_angle);  //对小电脑传输角度进行卡尔曼滤波

//	//滤波p轴
//    Data_Accelerated_Control(&vision_auto_data.pitch_control_data, 3.6f);                                                 //2.6                                                //斜坡加速度限制函数
//    vision_auto_data.pitch_control_data = Sliding_Mean_Filter(&pitch_mean_filt, vision_auto_data.pitch_control_data, 35); //均值滑窗滤波（有滞后）
//    pitch_data_filt = First_Order_Low_Filter(&pitch_low_filt, vision_auto_data.pitch_control_data);                       //一阶低通滤波
////	pitch_Kalman_filt = pitch_data_filt;
////	pitch_data_filt = KalmanFilter(&pitch_kalman_filt,pitch_data_filt);                                                         //卡尔曼滤波
////	Kalman_Filter_Calc(&kalman_Filter, vision_auto_data.auto_yaw_angle+Gimbal_yaw_angle, vision_auto_data.auto_pitch_angle);    //二阶卡尔曼滤波

//    //滤波y轴
//    yaw_data_filt = First_Order_Low_Filter(&yaw_low_filt, vision_auto_data.yaw_control_data); //一阶低通滤波

    /*云台视觉控制量*/ //(y轴，p轴还没用卡尔曼)
    //gimbal_auto_f->auto_c->pitch_control_data = gimbal_auto_f->auto_c->auto_pitch_angle * 1.7f;
    gimbal_auto_f->auto_c->pitch_control_data = gimbal_auto_f->auto_c->auto_pitch_angle * 5.0f;
    gimbal_auto_f->auto_c->yaw_control_data = gimbal_auto_f->auto_c->auto_yaw_angle;
	if (gimbal_auto_f->auto_c->auto_yaw_angle > 1.0f)
	{
		gimbal_auto_f->auto_c->yaw_control_data = (gimbal_auto_f->auto_c->auto_yaw_angle * gimbal_auto_f->auto_c->auto_yaw_angle);
	}

    /*记录pitch轴位置*/
    if (gimbal_auto_f->auto_c->pitch_control_data == 0.0f) // NO_4 修改增加
    {
        gimbal_auto_f->auto_c->auto_lost_data_count++;
		if (gimbal_auto_f->auto_c->auto_lost_data_count < 5)
		{
			gimbal_auto_f->auto_c->yaw_control_data = gimbal_auto_f->auto_c->last_Auto_Pitch_Angle;
		}
        else if (gimbal_auto_f->auto_c->auto_lost_data_count == 5) //约10ms
        {
            gimbal_auto_f->auto_c->auto_lost_data_count = 0;
            gimbal_auto_f->auto_c->auto_lost_data_flag = 1; //丢失目标
			gimbal_auto_f->auto_c->last_Auto_Pitch_Angle = 0;
        }
    }
    else
    {
        gimbal_auto_f->auto_c->auto_lost_data_count = 0;
        gimbal_auto_f->auto_c->auto_lost_data_flag = 0; //没有丢失目标

		//实时记录p轴数据
        gimbal_auto_f->pitch_c.Auto_record_location = gimbal_auto_f->pitch_c.pitch_motor_measure->actual_Position;
		gimbal_auto_f->auto_c->last_Auto_Pitch_Angle = gimbal_auto_f->auto_c->yaw_control_data;
    }

    /*记录yaw轴位置*/
    if (gimbal_auto_f->auto_c->yaw_control_data == 0.0f)
    {
        gimbal_auto_f->auto_c->auto_yaw_zero_flag++;
        if (gimbal_auto_f->auto_c->auto_yaw_zero_flag <= 5) //大约10ms+
        {
            gimbal_auto_f->auto_c->yaw_control_data = gimbal_auto_f->auto_c->last_Auto_Yaw_Angle;
        }
        else
        {
            gimbal_auto_f->auto_c->auto_yaw_zero_flag = 0;
            gimbal_auto_f->auto_c->yaw_control_data = 0.0f;
            gimbal_auto_f->auto_c->last_Auto_Yaw_Angle = 0.0f;
        }
    }
    else
    {
        gimbal_auto_f->auto_c->auto_yaw_zero_flag = 0;

        gimbal_auto_f->auto_c->last_Auto_Yaw_Angle = gimbal_auto_f->auto_c->yaw_control_data;
        gimbal_auto_f->yaw_c.last_angle = gimbal_auto_f->imu_c->yaw_angle;
    }

/* P轴滤波 */
	gimbal_auto_f->auto_c->pitch_control_data = first_order_filter(&gimbal_auto_f->pitch_c.LowFilt_auto_pitch, gimbal_auto_f->auto_c->pitch_control_data);       //一阶低通滤波
//	gimbal_auto_f->auto_c->pitch_control_data = sliding_mean_filter(&gimbal_auto_f->pitch_c.Slidmean_auto_pitch, gimbal_auto_f->auto_c->pitch_control_data, 35); //均值滑窗滤波（有滞后）
//	gimbal_auto_f->auto_c->yaw_control_data = first_order_filter(&gimbal_auto_f->pitch_c.LowFilt_Pitch_Data, gimbal_auto_f->auto_c->yaw_control_data);       //一阶低通滤波

    gimbal_automatic_work(gimbal_auto_f);
}

/**
  * @brief          云台无力
  * @param[in]      none
  * @retval         none
  * @attention
  */
void gimbal_ch_retain(gimbal_control_t *gimbal_ch_f)
{
    //Gimbal_ch3 = 0.0f; //pitch轴
    Gimbal_ch3 = gimbal_ch_f->pitch_c.pitch_motor_measure->actual_Position * 360 / 1024; // TODO

    Gimbal_ch2 = 0.0f; //yaw轴
}

/**
  * @brief          云台归中
  * @param[in]      none
  * @retval         none
  * @attention
  */
void gimbal_ch_set0(void)
{
    Gimbal_ch3 = 0.0f; //pitch轴
    Gimbal_ch2 = 0.0f; //yaw轴
}

/**
  * @brief          返回云台模式
  * @param[in]      none
  * @retval         
  */
const gimbal_behaviour_e *get_gimbal_behaviour_point(void)
{
    return (&gimbal_behaviour);
}


/**
 * @description: 抬升,有传感器
 * @param {*Motor_t *uplift,u8 dire :1 抬升  2 放下}
 * @return {*}
 */
static void uplift_behav(gimbal_control_t *uplift_motor, VL53L0_t *vl53l0, float dis, u8 dire)
{

    if(dire == 1)																													//向上
    {
        uplift_motor->uplift_c.speed_set = UpLift_Speed; 																		//初速度赋值

        if(vl53l0->distance - vl53l0->InitDistance >= dis)//结束标志
        {
						uplift_motor->uplift_c.last_angle = (uplift_motor->uplift_c.uplift_measure->actual_Position *360/8192/LIFT_RATIO);
            uplift_motor->uplift_c.complete_state = Finish;
        }
    }
    else if(dire == 2)																										//向下
    {
        uplift_motor->uplift_c.speed_set = -UpLift_Speed; //初速度赋值 测试

        if(vl53l0->distance - vl53l0->InitDistance <= dis)									//结束标志
        {
						uplift_motor->uplift_c.last_angle = (uplift_motor->uplift_c.uplift_measure->actual_Position *360/8192/LIFT_RATIO);
            uplift_motor->uplift_c.complete_state = Finish;
        }
    }
    else
        uplift_motor->uplift_c.speed_set = 0;
}

/**
 * @description: 平移
 * @param {Motor_t} *telescoping
 * @param {u8} dire
 * @return {*}
 * @note 靠堵转试试    2--后伸  1--前伸
 */
static void translation_behav(gimbal_control_t *translation_motor, u8 dire)
{
    static uint8_t clock = 0 ;

    if(dire == 1 || dire == 2)
    {
        translation_motor->telescoping_c.speed_set = (dire * 2 - 3) * Telescoping_Speed;

        if (int16_t_abs(translation_motor->telescoping_c.telescoping_measure->speed) <= 300)
        {
            clock++;

            if (clock > 5)
            {
                clock = 0;
                translation_motor->telescoping_c.last_angle = (translation_motor->telescoping_c.telescoping_measure->actual_Position *360/8192/RESCUE_RATIO);
                translation_motor->telescoping_c.complete_state = Finish;
            }
        }
    }
    else
    {
        translation_motor->telescoping_c.last_angle = (translation_motor->telescoping_c.telescoping_measure->actual_Position *360/8192/RESCUE_RATIO);
    }
}

/**
 * @description: 伸缩
 * @param {Motor_t} *telescoping
 * @param {u8} dire
 * @return {*}
 * @note 靠堵转试试    2--后伸  1--前伸
 */
static void telescoping_behav(gimbal_control_t *telescoping_motor, u8 dire)
{
    static uint8_t clock = 0 ;

    if(dire == 1 || dire == 2)
    {
        telescoping_motor->telescoping_c.speed_set = (dire * 2 - 3) * Telescoping_Speed*0.3f;

        if (int16_t_abs(telescoping_motor->telescoping_c.telescoping_measure->speed) <= 50)
        {
            clock++;

            if (clock > 20)
            {
                clock = 0;
                telescoping_motor->telescoping_c.last_angle = (telescoping_motor->telescoping_c.telescoping_measure->actual_Position *360/8192/FUNCTION_RATIO);
                telescoping_motor->telescoping_c.complete_state = Finish;
            }
        }
    }
    else
    {
        telescoping_motor->telescoping_c.last_angle = (telescoping_motor->telescoping_c.telescoping_measure->actual_Position *360/8192/FUNCTION_RATIO);
    }
}

/**
 * @description: 夹子
 * @param {Motor_t} *clip
 * @param {int16_t} exp  : expect Speed
 * @param {u8} dire      : 1 夹紧   2 松开
 * @return {*}
 */
static void clip_behav(gimbal_control_t *clip_motor, u8 dire)
{
    static int16_t clock = 0;

    if(dire == 1 || dire == 2)
    {
        clip_motor->clip_c.speed_set = (3 - dire * 2) * Cilp_Speed;

        /*结束判断*/
        if(int16_t_abs(clip_motor->clip_c.clip_measure->speed) <= 50)
        {
            clock ++;

            if(clock > 100)
            {
                clock = 0;
                clip_motor->clip_c.last_angle = (clip_motor->clip_c.clip_measure->actual_Position *360/8192/FUNCTION_RATIO);
                clip_motor->clip_c.complete_state = Finish;
            }
        }
    }
    else
    {
       clip_motor->clip_c.last_angle = (clip_motor->clip_c.clip_measure->actual_Position *360/8192/FUNCTION_RATIO);
    }
}

/**
  * @description: 使得电机转过绝对角度
  * @param {Motor_t} *filp1
  * @param {float} exp
  * @param {float} limit
  * @param {u8} dire 1-前翻  2-后翻
  * @return {*}
  */
static void flip_behav(gimbal_control_t *filp_motor, float exp, float limit, u8 dire)
{
    static int16_t clock = 0;

    if (dire == 1)
    {

        filp_motor->flip_c.position_set -= 0.4f;

        if(filp_motor->flip_c.position_set <= (filp_motor->flip_c.angle_init - exp))
        {
            //     x <= 6-182 =
            filp_motor->flip_c.position_set = (filp_motor->flip_c.angle_init - exp);
        }

        /*完成标志*/
        // 平行： -151 <=18-exp  ===>exp = 169
        // 垂直： -225 + exp -6 <=6  ===> exp = 237
        if(float_abs((filp_motor->flip_c.flip_measure->actual_Position* 360/8192/FUNCTION_RATIO) + exp - filp_motor->flip_c.angle_init) <= limit)
        {
            filp_motor->clip_c.complete_state = Finish;
            filp_motor->flip_c.position_set = (filp_motor->flip_c.angle_init - exp);
        }

    }
    else if (dire == 2)
    {
        filp_motor->flip_c.position_set += 0.3f;


        /*完成标志*/
        if (float_abs((filp_motor->flip_c.flip_measure->actual_Position* 360/8192/FUNCTION_RATIO) - filp_motor->flip_c.angle_init) <= limit)
        {
            clock ++;
        }

        if (clock >= 20)
        {
            filp_motor->flip_c.position_set = (filp_motor->flip_c.flip_measure->actual_Position* 360/8192/FUNCTION_RATIO);
            filp_motor->flip_c.complete_state = Finish;
            clock = 0;
        }
    }
    else
    {
        filp_motor->flip_c.position_set = (filp_motor->flip_c.flip_measure->actual_Position* 360/8192/FUNCTION_RATIO);
    }
}

/**
 * @description:夹取金矿初始化
 * @param {Gr_t} *Gr
 * @param
 * @return {*}
 * @note 201-抬升	202-平移	203-伸缩	204-夹子	205-翻转	206-翻转	207-弹仓
 * 	 0 抬升   2伸缩   3 夹紧  4 翻转  7旋转
 * @note ：此函数为向上抬与往前伸 抬升具体的高度还要再测
 */
static void grasp_init(gimbal_control_t *grasp_motor_init)
{
    switch (grasp_motor_init->robo_state)
    {
        /*抬升*/
        case 0:
        {
            grasp_motor_init->uplift_c.complete_state = DisFinish;//预备抬升

            uplift_behav(grasp_motor_init, grasp_motor_init->vL53L0, 20.0f, 1);

            if(grasp_motor_init->uplift_c.complete_state == Finish)
            {
                grasp_motor_init->translation_c.complete_state = DisFinish;//预备平移
                grasp_motor_init->flip_c.complete_state = DisFinish;//预备夹取
                grasp_motor_init->robo_state = 1;
            }

            break;
        }

        /*前移 翻转*/
        case 1:
        {
            translation_behav(grasp_motor_init, 1);//平移
						flip_behav(grasp_motor_init, 80, 10, 1);//翻转至90°

            if(grasp_motor_init->translation_c.complete_state == Finish && grasp_motor_init->flip_c.complete_state == Finish)
            {
                grasp_motor_init->telescoping_c.complete_state = DisFinish;//预备夹取
                grasp_motor_init->robo_state = 2;
            }

            break;
        }

        /*前伸*/
        case 2:
        {

						telescoping_behav(grasp_motor_init, 1);

						if(grasp_motor_init->telescoping_c.complete_state == Finish)
						{
								grasp_motor_init->flip_c.complete_state = DisFinish;
								grasp_motor_init->robo_state = 3;
						}
					}

            break;
        
				
        default:
            break;
    }

}

static void grasp_reset(gimbal_control_t *grasp_motor_reset)
{
		grasp_motor_reset->uplift_c.complete_state = DisFinish;//预备下降
	
		uplift_behav(grasp_motor_reset, grasp_motor_reset->vL53L0, 1.0f, 2);
	
		if(grasp_motor_reset->uplift_c.complete_state == Finish)
		{
				grasp_motor_reset->translation_c.complete_state = DisFinish;//预备平移
        grasp_motor_reset->flip_c.complete_state = DisFinish;//预备夹取
		}
		
		if(grasp_motor_reset->translation_c.complete_state == Finish)
		{
				translation_behav(grasp_motor_reset, 2);//预备平移
		}
		
		if(grasp_motor_reset->flip_c.complete_state == Finish)
		{
				flip_behav(grasp_motor_reset, 0, 10, 2);//翻转至初始
		}
		grasp_motor_reset->robo_state = 0;
}


static void grasp_first(gimbal_control_t *grasp_motor_first)
{
    switch (grasp_motor_first->robo_state)
    {
        /*前翻*/
        case 3:
        {
					if(grasp_motor_first->sensor->Smooth_L && grasp_motor_first->sensor->Smooth_R)
          {
            flip_behav(grasp_motor_first, 170, 10, 1);

            if(grasp_motor_first->flip_c.complete_state == Finish)
            {
                grasp_motor_first->clip_c.complete_state = DisFinish;
                grasp_motor_first->robo_state = 4;
            }
					}
            break;
        }

        /*夹紧*/
        case 4:
        {
            clip_behav(grasp_motor_first, 1);

            if(grasp_motor_first->clip_c.complete_state == Finish)
            {
                grasp_motor_first->flip_c.complete_state = DisFinish;
                grasp_motor_first->robo_state = 5;
            }

            break;
        }

        /*后翻*/
        case 5:
        {
            flip_behav(grasp_motor_first, 0, 10, 2);

            if(grasp_motor_first->flip_c.complete_state == Finish)
            {
                grasp_motor_first->telescoping_c.complete_state = DisFinish;
                grasp_motor_first->robo_state = 6;
            }

            break;
        }

        /*后伸*/
        case 6:
        {
            telescoping_behav(grasp_motor_first, 2);

            if(grasp_motor_first->telescoping_c.complete_state == Finish)
            {
                grasp_motor_first->clip_c.complete_state = DisFinish;
                grasp_motor_first->robo_state = 7;
            }

            break;
        }

        /*松夹*/
        case 7:
        {
            clip_behav(grasp_motor_first, 2);

            if(grasp_motor_first->clip_c.complete_state == Finish)
            {
                grasp_motor_first->translation_c.complete_state = DisFinish;
                grasp_motor_first->flip_c.complete_state = DisFinish;
                grasp_motor_first->robo_state = 8;
            }

            break;
        }

        /*后移 翻转*/
        case 8:
        {
            translation_behav(grasp_motor_first, 2);//预备平移
            flip_behav(grasp_motor_first, 65, 10, 1);//预备翻转至90°

            if(grasp_motor_first->translation_c.complete_state == Finish && grasp_motor_first->flip_c.complete_state == Finish)
            {
                grasp_motor_first->robo_state = 2;//复位，等待下一条夹取指令
            }

            break;
        }

        default:
            break;
    }

}

static void grasp_second(gimbal_control_t *grasp_motor_second)
{
    switch (grasp_motor_second->robo_state)
    {
        /*初始化完成了才能进行夹取*/
        /*前翻*/
        case 3:
        {
            if(grasp_motor_second->sensor->Smooth_L && grasp_motor_second->sensor->Smooth_R)
            {
								flip_behav(grasp_motor_second, 170, 10, 1);

								if(grasp_motor_second->flip_c.complete_state == Finish)
								{
										grasp_motor_second->clip_c.complete_state = DisFinish;
										grasp_motor_second->robo_state = 4;
								}
						 }
            break;
        }

        /*夹紧*/
        case 4:
        {
            clip_behav(grasp_motor_second, 1);

            if(grasp_motor_second->clip_c.complete_state == Finish)
            {
                grasp_motor_second->telescoping_c.complete_state = DisFinish;
                grasp_motor_second->robo_state = 5;
            }
            break;
        }

        /*后伸*/
        case 5:
        {
            telescoping_behav(grasp_motor_second, 2);

            if(grasp_motor_second->telescoping_c.complete_state == Finish)
            {
                grasp_motor_second->flip_c.complete_state = DisFinish;
                grasp_motor_second->robo_state = 6;
            }

            break;
        }

        /*后翻*/
        case 6:
        {
            flip_behav(grasp_motor_second, 0, 10, 2);

            if(grasp_motor_second->flip_c.complete_state == Finish)
            {
                grasp_motor_second->clip_c.complete_state = DisFinish;
                grasp_motor_second->robo_state = 7;
            }

            break;
        }

        /*松夹*/
        case 7:
        {
            clip_behav(grasp_motor_second, 2);

            if(grasp_motor_second->clip_c.complete_state == Finish)
            {
                grasp_motor_second->flip_c.complete_state = DisFinish;
                grasp_motor_second->robo_state = 8;
            }

            break;
        }

        /*翻转*/
        case 8:
        {
            flip_behav(grasp_motor_second, 65, 10, 1);//预备翻转至90°

            if(grasp_motor_second->flip_c.complete_state == Finish)
            {
                grasp_motor_second->robo_state = 2;//复位，等待下一条夹取指令
            }
            break;
        }


        default:
            break;
    }

}


static void grasp_third(gimbal_control_t *grasp_motor_third)
{
    switch (grasp_motor_third->robo_state)
    {
        /*初始化完成了才能进行夹取*/
        /*前翻*/
        case 3:
        {
            if(grasp_motor_third->sensor->Smooth_L && grasp_motor_third->sensor->Smooth_R)
            {
								flip_behav(grasp_motor_third, 170, 10, 1);

								if(grasp_motor_third->flip_c.complete_state == Finish)
								{
										grasp_motor_third->clip_c.complete_state = DisFinish;
										grasp_motor_third->robo_state = 4;
								}
            }

            break;
        }

        /*夹紧*/
        case 4:
        {
            clip_behav(grasp_motor_third, 1);

            if(grasp_motor_third->clip_c.complete_state == Finish)
            {
                grasp_motor_third->telescoping_c.complete_state = DisFinish;
                grasp_motor_third->robo_state = 5;
            }

            break;
        }

        /*后伸*/
        case 5:
        {
            telescoping_behav(grasp_motor_third, 2);

            if(grasp_motor_third->telescoping_c.complete_state == Finish)
            {
                grasp_motor_third->flip_c.complete_state = DisFinish;
                grasp_motor_third->robo_state = 6;
            }
            break;
        }

        /*后翻*/
        case 6:
        {
            flip_behav(grasp_motor_third, 35, 10, 1);

            if(grasp_motor_third->flip_c.complete_state == Finish)
            {
                grasp_motor_third->robo_state = 2;//复位，等待下一条夹取指令
            }

            break;
        }

        default:
            break;
    }

}

static void flap_reset(gimbal_control_t *flap_motor_reset)
{
    switch (flap_motor_reset->robo_state)
    {
				case 3:
				{
						flip_behav(flap_motor_reset, 80, 10, 1);//翻转至90°
					
						
					break;
				}
        default:
            break;
		}
}



