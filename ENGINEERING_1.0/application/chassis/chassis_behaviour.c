/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       chassis_task.c/h
  * @brief      完成底盘控制任务
  * @note       合并版本
  * @history    2021.07.05
  *
  @verbatim   
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
  */
#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "rc.h"
#include "chassis_behaviour.h"
#include "rm_motor.h"
#include "referee_deal.h"
#include "delay.h"
#include "parameter.h"
#include "system_state.h"

static void mk_data_process(chassis_control_t *MK_Data_Process_f);                                       //键盘鼠标选择
static void chassis_mode_choose(chassis_control_t *chassis_mode_choose_f);                               //底盘模式选择
static void chassis_follow(chassis_control_t *Chassis_Follow_f);                                         //底盘跟随
static void chassis_twist(chassis_control_t *Chassis_Twist_f, fp32 Before_After, fp32 Left_Right);       //底盘扭腰
static void chassis_rotation(chassis_control_t *Chassis_Rotation_f, fp32 Before_After, fp32 Left_Right); //底盘小陀螺
static void chassis_independent(chassis_control_t *Chassis_Independent_f);                               //底盘不跟随
static void chassis_rescue(chassis_control_t *Rescue_Card ,int16_t dire);
static void rescue_clap(chassis_control_t *Rescue_Clap ,int16_t dire);


chassis_behaviour_e chassis_behaviour;
rescuemotor_behaviour_e rescue_behaviour;
float Chassis_ch0 = 0.0f, Chassis_ch1 = 0.0f, Chassis_ch2 = 0.0f; //底盘电机受控量
float Chy = 0.0f, Chx = 0.0f;
float rescuemotorspeed_set_value = 0.0f;
float rescueclapspeed_set_value = 0.0f;
float rescuemotor_speed_set = 0.0f;
float rescueclap_speed_set = 0.0f;
int8_t Vision_flag = 0;                                           //视觉模式切换标志（0：关闭，1：自瞄，2：能量机关）
//static int16_t Speed_W_ACC = 0;
//static int16_t Speed_S_ACC = 0;
//static int16_t Speed_A_ACC = 0;
//static int16_t Speed_D_ACC = 0;

int chassis_cos_calculate_jscope = 0;
int chassis_sin_calculate_jscope = 0;

/**
  * @brief          底盘模式设置
  * @param[in]      *chassis_pid_f：底盘主结构体
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f)
{
    //!底盘模式选择
    chassis_mode_choose(chassis_behaviour_f);

    if ((system_state_return() != INF_STOP))
    {
        chassis_behaviour_f->chassis_speed_pid[0].pid_mode = PID_NORMAL;
        chassis_behaviour_f->chassis_speed_pid[1].pid_mode = PID_NORMAL;
        chassis_behaviour_f->chassis_speed_pid[2].pid_mode = PID_NORMAL;
        chassis_behaviour_f->chassis_speed_pid[3].pid_mode = PID_NORMAL;

        //*键盘
        if (system_state_return() == INF_MK)
        {
            if (chassis_behaviour == CHASSIS_FOLLOW) //*跟随
            {
                Chassis_ch0 = chassis_behaviour_f->key.key_a + chassis_behaviour_f->key.key_d;
                Chassis_ch1 = chassis_behaviour_f->key.key_w + chassis_behaviour_f->key.key_s;
                //获取云台与底盘的差角
                Chassis_ch2 = chassis_behaviour_f->key.mousex;
								if(Chassis_ch2 == 0 && ((Chassis_ch0 !=0) || (Chassis_ch1 !=0)))
								{
										chassis_behaviour_f->chassis_rotate_pid.SetValue = -chassis_behaviour_f->gyro->Yaw_Var;
								}
								else
								{
										chassis_behaviour_f->gyro->Yaw_Lock = chassis_behaviour_f->gyro->Yaw_Var;
								}
                //将跟随底盘的pid目标值设定为差角
                chassis_behaviour_f->chassis_rotate_pid.SetValue = Chassis_ch2;
                //将经过pid计算的输出值传给 Chassis_ch2
                Chassis_ch2 = location_pid_int32(&chassis_behaviour_f->chassis_rotate_pid, 0.0f);
            }
            else if (chassis_behaviour == CHASSIS_BATTERY) //*炮台模式（底盘位置环锁死
            {
                Chassis_ch0 = Chassis_ch1 = Chassis_ch2 = 0;
            }
            else if (chassis_behaviour == CHASSIS_TWIST_WAIST) //*扭腰
            {
                chassis_twist(chassis_behaviour_f, chassis_behaviour_f->key.key_w + chassis_behaviour_f->key.key_s, chassis_behaviour_f->key.key_a + chassis_behaviour_f->key.key_d);
            }
            else if (chassis_behaviour == CHASSIS_ROTATION) //*小陀螺
            {
                chassis_rotation(chassis_behaviour_f, chassis_behaviour_f->key.key_w + chassis_behaviour_f->key.key_s, chassis_behaviour_f->key.key_a + chassis_behaviour_f->key.key_d);
            }
						
						if(rescue_behaviour == RESCUE_FORWARD)
						{
								chassis_rescue(chassis_behaviour_f, 2);
						}
						else if(rescue_behaviour == RESCUE_BACK)
						{
								chassis_rescue(chassis_behaviour_f, 1);
						}
						if(rescue_behaviour == RESCLAP_FORWARD)
						{
								rescue_clap(chassis_behaviour_f, 2);
						}
						else if(rescue_behaviour == RESCLAP_BACK)
						{
								rescue_clap(chassis_behaviour_f, 1);
						}
        }
        else
        {
            if (chassis_behaviour == CHASSIS_FOLLOW)
            {
                chassis_follow(chassis_behaviour_f);
            }
            else if (chassis_behaviour == CHASSIS_NO_FOLLOW)
            {
                chassis_independent(chassis_behaviour_f);
            }
            else if (chassis_behaviour == CHASSIS_TWIST_WAIST)
            {
                chassis_twist(chassis_behaviour_f, chassis_behaviour_f->chassis_RC->rc.ch[1], chassis_behaviour_f->chassis_RC->rc.ch[0]);
            }
            else if (chassis_behaviour == CHASSIS_ROTATION)
            {
                chassis_rotation(chassis_behaviour_f, chassis_behaviour_f->chassis_RC->rc.ch[1], chassis_behaviour_f->chassis_RC->rc.ch[0]);
            }
        }
				Chy = chassis_behaviour_f->key.mousey;
				Chx = chassis_behaviour_f->key.press;
        //*传入值处理
        chassis_set_remote(chassis_behaviour_f, Chassis_ch0, Chassis_ch1, Chassis_ch2);
				sgimbal_set_remote(chassis_behaviour_f, Chx, Chy);
				rescue_set_remote(chassis_behaviour_f, rescuemotorspeed_set_value, rescueclapspeed_set_value);
    }
    else
    {
        chassis_behaviour_f->chassis_speed_pid[0].pid_mode = PID_MANUAL;
        chassis_behaviour_f->chassis_speed_pid[1].pid_mode = PID_MANUAL;
        chassis_behaviour_f->chassis_speed_pid[2].pid_mode = PID_MANUAL;
        chassis_behaviour_f->chassis_speed_pid[3].pid_mode = PID_MANUAL;
        chassis_task_off(0);
        chassis_set_remote(chassis_behaviour_f, 0.0f, 0.0f, 0.0f);
				sgimbal_set_remote(chassis_behaviour_f, 0.0f, 0.0f);
        remote_reload(); //摇杆量清零
    }
}

/**
  * @brief          底盘模式选择
  * @param[in]      *chassis_mode_choose_f：底盘主结构体
  * @retval         none
  */
static void chassis_mode_choose(chassis_control_t *chassis_mode_choose_f)
{
    /* 右开关打上，遥控控制 */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW_UP)
    {
        if (chassis_mode_choose_f->gimbal_re_data->Gimbal_all_flag == 1)
        {
            system_state_set(INF_RC);
            if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW_UP) //s1（左上）打到上
            {
                chassis_behaviour = CHASSIS_ROTATION; //底盘小陀螺模式
            }
            else if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW_MID) //s1（左上）打到中间
            {
                chassis_behaviour = CHASSIS_FOLLOW; //底盘跟随模式
            }
            else if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW_DOWN) //s1（左上）打到最下
            {
                chassis_behaviour = CHASSIS_NO_FOLLOW; //不跟随
            }
        }
    }
    /* 右开关打中，键盘控制 */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW_MID)
    {
//        if (chassis_mode_choose_f->gimbal_re_data->Gimbal_all_flag == 1)
        {
            //开关切换模式
            if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW_UP) //s1（左上）打到上
            {
                system_state_set(INF_RC);
            }
            else if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW_MID) //s1（左上）打到中间
            {
                if (system_state_return() != INF_MK)
                {
                    chassis_behaviour = CHASSIS_FOLLOW; //状态设置为跟随
                }
                system_state_set(INF_MK);
                mk_data_process(chassis_mode_choose_f); //!键盘模式
            }
            else if (chassis_mode_choose_f->chassis_RC->rc.s1 == RC_SW_DOWN) //s1（左上）打到最下
            {
                chassis_behaviour = CHASSIS_BATTERY; //炮台模式
                system_state_set(INF_RC);
            }
        }
    }
    /* 右开关打下，停止工作 */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW_DOWN)
    {
        chassis_behaviour = CHASSIS_ZERO_FORCE;
				rescue_behaviour = RESCUE_BACK;
        system_state_set(INF_STOP); //进入停止状态
    }
    /* 出现严重错误 */
    if (chassis_mode_choose_f->chassis_RC->rc.s2 == RC_SW_ERROR)
    {
        //chassis_mode_choose_f->chassis_mode = CHASSIS_STANDBY; //进入待机模式
    }
}

/**
  * @brief          键盘鼠标选择
  * @param[in]      *MK_Data_Process_f：底盘主结构体
  * @retval         none
  * @attention
  */
void mk_data_process(chassis_control_t *MK_Data_Process_f)
{
    /* 基本运动 */
    {
        //**W 前
        if (MK_Data_Process_f->chassis_RC->kb.bit.W)
        {
            MK_Data_Process_f->key.key_w = NORMAL_FORWARD_BACK_SPEED;
			MK_Data_Process_f->key.key_w = (1-CHASSIS_MK_SECOND_FILTERING) * MK_Data_Process_f->key.last_key_w + CHASSIS_MK_SECOND_FILTERING * MK_Data_Process_f->key.key_w;
        }
        else
        {
            MK_Data_Process_f->key.key_w = 0.0f;
			MK_Data_Process_f->key.last_key_w = 0.0f;
        }
        //**S 后
        if (MK_Data_Process_f->chassis_RC->kb.bit.S)
        {
            MK_Data_Process_f->key.key_s = -NORMAL_FORWARD_BACK_SPEED;
			MK_Data_Process_f->key.key_s = (1-CHASSIS_MK_SECOND_FILTERING) * MK_Data_Process_f->key.last_key_s + CHASSIS_MK_SECOND_FILTERING * MK_Data_Process_f->key.key_s;
        }
        else
        {
            MK_Data_Process_f->key.key_s = 0.0f;
			MK_Data_Process_f->key.last_key_s = 0.0f;
        }
        //**A 左
        if (MK_Data_Process_f->chassis_RC->kb.bit.A)
        {
            MK_Data_Process_f->key.key_a = NORMAL_LEFT_RIGHT_SPEED;
			MK_Data_Process_f->key.key_a = (1-CHASSIS_MK_SECOND_FILTERING) * MK_Data_Process_f->key.last_key_a + CHASSIS_MK_SECOND_FILTERING * MK_Data_Process_f->key.key_a;
        }
        else
        {
            MK_Data_Process_f->key.key_a = 0.0f;
			MK_Data_Process_f->key.last_key_a = 0.0f;
        }
        //**D 右
        if (MK_Data_Process_f->chassis_RC->kb.bit.D)
        {
            MK_Data_Process_f->key.key_d = -NORMAL_LEFT_RIGHT_SPEED;
			MK_Data_Process_f->key.key_d = (1-CHASSIS_MK_SECOND_FILTERING) * MK_Data_Process_f->key.last_key_d + CHASSIS_MK_SECOND_FILTERING * MK_Data_Process_f->key.key_d;
        }
        else
        {
            MK_Data_Process_f->key.key_d = 0.0f;
			MK_Data_Process_f->key.last_key_d = 0.0f;
        }
				//**左右转
        if (MK_Data_Process_f->chassis_RC->mouse.x)
        {
            MK_Data_Process_f->key.mousex = MK_Data_Process_f->chassis_RC->mouse.x*NORMAL_LEFT_RIGHT_SPEED*0.002f;
			MK_Data_Process_f->key.mousex = (1-CHASSIS_MK_SECOND_FILTERING) * MK_Data_Process_f->key.last_mousex + CHASSIS_MK_SECOND_FILTERING * MK_Data_Process_f->key.mousex;
        }
        else
        {
            MK_Data_Process_f->key.mousex = 0.0f;
			MK_Data_Process_f->key.last_mousex = 0.0f;
        }
				//**小云台
        if (MK_Data_Process_f->chassis_RC->mouse.press_l)
        {
            MK_Data_Process_f->key.press -= 0.6f;
        }
				else if(MK_Data_Process_f->chassis_RC->mouse.press_r)
				{
						MK_Data_Process_f->key.press += 0.6f;
				}
        else
        {
            MK_Data_Process_f->key.press = 0.0f;
        }
        if (MK_Data_Process_f->chassis_RC->mouse.y)
        {
            MK_Data_Process_f->key.mousey -= MK_Data_Process_f->chassis_RC->mouse.y *0.01f *0.6f;
						MK_Data_Process_f->key.mousey  = float_limit(MK_Data_Process_f->key.mousey,25,-25);
        }
    }

    //**Shife 跟随模式
    if (MK_Data_Process_f->chassis_RC->kb.bit.SHIFT)
    {
        chassis_behaviour = CHASSIS_FOLLOW;
    }

    //**Ctrl 炮台模式
    if (MK_Data_Process_f->chassis_RC->kb.bit.CTRL)
    {
        chassis_behaviour = CHASSIS_BATTERY; //炮台模式
    }
    else if (chassis_behaviour == CHASSIS_BATTERY)
    {
		chassis_behaviour = CHASSIS_FOLLOW; //跟随模式
    }

    //**Z 陀螺模式
    if (MK_Data_Process_f->chassis_RC->kb.bit.Z)
    {
				chassis_behaviour = CHASSIS_ROTATION; //小陀螺模式
    }
    //**X 扭腰模式
    if (MK_Data_Process_f->chassis_RC->kb.bit.X)
    {
        chassis_behaviour = CHASSIS_TWIST_WAIST; //底盘扭腰模式
    }
    //**C 自动模式
    if (MK_Data_Process_f->chassis_RC->kb.bit.C)
    {
		
    }
		
    //**Q
    if (MK_Data_Process_f->chassis_RC->kb.bit.Q)
    {
    }

    //**E
    if (MK_Data_Process_f->chassis_RC->kb.bit.E)
    {
		
    }

    //**R
    if (MK_Data_Process_f->chassis_RC->kb.bit.R)
    {

    }

    //**F 救援卡
    if (MK_Data_Process_f->chassis_RC->kb.bit.F)
    {
				if(MK_Data_Process_f->chassis_RC->kb.bit.F && MK_Data_Process_f->chassis_RC->kb.bit.CTRL)
		{
				rescue_behaviour = RESCUE_BACK;
		}
		else
			{
        rescue_behaviour = RESCUE_FORWARD;
			}
    }
    //**G 救援爪
    if (MK_Data_Process_f->chassis_RC->kb.bit.G)
    {
				if(MK_Data_Process_f->chassis_RC->kb.bit.G && MK_Data_Process_f->chassis_RC->kb.bit.CTRL)
				{
									rescue_behaviour = RESCLAP_BACK;
				}
				else
				{
				rescue_behaviour = RESCLAP_FORWARD;
				}
		}
}


/**
  * @brief          底盘跟随状态控制
  * @param[in]      *Chassis_Follow_f：底盘主结构体
  * @retval         none
  */
static void chassis_follow(chassis_control_t *Chassis_Follow_f)
{
    Chassis_ch0 = Chassis_Follow_f->chassis_RC->rc.ch[0];
    Chassis_ch1 = Chassis_Follow_f->chassis_RC->rc.ch[1];

    //获取云台与底盘的差角
    Chassis_ch2 = Chassis_Follow_f->chassis_gimbal_angel;

    //将跟随底盘的pid目标值设定为差角
    Chassis_Follow_f->chassis_rotate_pid.SetValue = Chassis_ch2;
    //将经过pid计算的输出值传给 Chassis_ch2
    Chassis_ch2 = location_pid_int32(&Chassis_Follow_f->chassis_rotate_pid, 0.0f);
    Chassis_ch2 = int32_limit(Chassis_ch2, 1000, -1000); //输出限制   //TODO

    //	Chassis_ch2 = Chassis_Follow_f->chassis_gimbal_angel * 3.5f;
}

/**
  * @brief          底盘不跟随状态控制
  * @param[in]      *Chassis_Independent_f：底盘主结构体
  * @retval         none
  */
static void chassis_independent(chassis_control_t *Chassis_Independent_f)
{
#if 1
    /* 小陀螺旋转移动算法：云台为主坐标轴，目标值分解到底盘坐标轴 */
    if (Chassis_Independent_f->chassis_RC->rc.ch[0] || Chassis_Independent_f->chassis_RC->rc.ch[1]) //此时有移动    custom_sin  sin_calculate()
    {
        //以步兵建立坐标轴，左负右正
        Chassis_ch2 = 0;

        Chassis_ch1 = cos_calculate(Chassis_Independent_f->chassis_gimbal_angel) * Chassis_Independent_f->chassis_RC->rc.ch[1];
        Chassis_ch0 = sin_calculate(Chassis_Independent_f->chassis_gimbal_angel) * Chassis_Independent_f->chassis_RC->rc.ch[1];

        if (Chassis_Independent_f->chassis_RC->rc.ch[0])
        {
            Chassis_ch1 += -sin_calculate(Chassis_Independent_f->chassis_gimbal_angel) * Chassis_Independent_f->chassis_RC->rc.ch[0];
            Chassis_ch0 += cos_calculate(Chassis_Independent_f->chassis_gimbal_angel) * Chassis_Independent_f->chassis_RC->rc.ch[0];
        }
    }
    else
    {
        Chassis_ch0 = 0;
        Chassis_ch1 = 0;
    }

    chassis_cos_calculate_jscope = cos_calculate(Chassis_Independent_f->chassis_gimbal_angel) * 100;
    chassis_sin_calculate_jscope = sin_calculate(Chassis_Independent_f->chassis_gimbal_angel) * 100;
#else
    Chassis_ch0 = Chassis_Independent_f->chassis_RC->rc.ch[0];
    Chassis_ch1 = Chassis_Independent_f->chassis_RC->rc.ch[1];
    Chassis_ch2 = Chassis_Independent_f->chassis_RC->rc.ch[2];
#endif
}

/**
  * @brief          底盘扭腰状态控制
  * @param[in]      *Chassis_Twist_f：底盘主结构体
  * @retval         none
  */
static int16_t TWIST_Flag = 0;        //0,左转；1，右转
static int16_t TWIST_Chassis_acc = 0; //底盘扭腰加速度限制
static void chassis_twist(chassis_control_t *Chassis_Twist_f, fp32 Before_After, fp32 Left_Right)
{
    //*判断转向
    if (Chassis_Twist_f->gyro->Yaw_Var > 25 && TWIST_Flag == 1)
    {
        TWIST_Flag = 0;
    }
    else if (Chassis_Twist_f->gyro->Yaw_Var < -25 && TWIST_Flag == 0)
    {
        TWIST_Flag = 1;
    }

    //*转动控制
    if (TWIST_Flag == 0) //左角度大于30，向左转
    {
        //*开环固定扭速
        if (TWIST_Chassis_acc <= CHASSIS_TWIST_SPEED)
            TWIST_Chassis_acc += 20;
        Chassis_ch2 = TWIST_Chassis_acc;

        //*扭腰移动算法：云台为主坐标轴，目标值分解到底盘坐标轴
        if (Before_After || Left_Right)
        {
            Chassis_ch1 = cos_calculate(Chassis_Twist_f->gyro->Yaw_Var) * Before_After;
            Chassis_ch0 = sin_calculate(Chassis_Twist_f->gyro->Yaw_Var) * Before_After;

            Chassis_ch1 += -sin_calculate(Chassis_Twist_f->gyro->Yaw_Var) * Left_Right;
            Chassis_ch0 += cos_calculate(Chassis_Twist_f->gyro->Yaw_Var) * Left_Right;
        }
        else
        {
            Chassis_ch0 = 0;
            Chassis_ch1 = 0;
        }
    }
    else if (TWIST_Flag == 1) //右角度小于负30，向右转
    {
        //*开环固定扭速
        if (TWIST_Chassis_acc >= -CHASSIS_TWIST_SPEED)
            TWIST_Chassis_acc -= 20;
        Chassis_ch2 = TWIST_Chassis_acc;

        //*扭腰移动算法：云台为主坐标轴，目标值分解到底盘坐标轴
        if (Before_After || Left_Right)
        {
            Chassis_ch1 = cos_calculate(Chassis_Twist_f->gyro->Yaw_Var) * Before_After;
            Chassis_ch0 = sin_calculate(Chassis_Twist_f->gyro->Yaw_Var) * Before_After;

            Chassis_ch1 += -sin_calculate(Chassis_Twist_f->gyro->Yaw_Var) * Left_Right;
            Chassis_ch0 += cos_calculate(Chassis_Twist_f->gyro->Yaw_Var) * Left_Right;
        }
        else
        {
            Chassis_ch0 = 0;
            Chassis_ch1 = 0;
        }
    }
    else
    {
        Chassis_ch0 = 0;
        Chassis_ch1 = 0;
        Chassis_ch2 = 0;
    }
}

/**
  * @brief          底盘小陀螺状态控制
  * @param[in]      *Chassis_Rotation_f：底盘主结构体
  * @retval         none
  */
fp32 obtain_rotate_motor_gain(void);
static void chassis_rotation(chassis_control_t *Chassis_Rotation_f, fp32 Before_After, fp32 Left_Right)
{
//    fp32 gain = obtain_rotate_motor_gain(); //TODO

    //*小陀螺旋转移动算法：云台为主坐标轴，目标值分解到底盘坐标轴
    if (Before_After || Left_Right) //此时有移动    custom_sin  sin_calculate()
    {
        //*以步兵建立坐标轴，左负右正
        Chassis_ch2 = CHASSIS_ROTATION_SPEED * 1.0f; //小陀螺移动时减速 0.8

        Chassis_ch1 = cos_calculate(Chassis_Rotation_f->gyro->Yaw_Var) * Before_After;
        Chassis_ch0 = sin_calculate(Chassis_Rotation_f->gyro->Yaw_Var) * Before_After;

        if (Left_Right)
        {
            Chassis_ch1 += -sin_calculate(Chassis_Rotation_f->gyro->Yaw_Var) * Left_Right;
            Chassis_ch0 += cos_calculate(Chassis_Rotation_f->gyro->Yaw_Var) * Left_Right;
        }
    }
    else //原地转快速
    {
        //*开环固定转速
        Chassis_ch2 = CHASSIS_ROTATION_SPEED * 1.0f; 

        Chassis_ch0 = 0;
        Chassis_ch1 = 0;
    }
}

fp32 obtain_rotate_motor_gain(void)
{
    uint16_t power = referee_chassis_power_limit();

    if (power == 40)
    {
        return 3.0f;
    }
    else if (power == 45)
    {
        return 3.5f;
    }
    else if (power == 50)
    {
        return 4.5f;
    }
    else if (power == 55)
    {
        return 4.5f;
    }
    else if (power == 60)
    {
        return 5.0f;
    }
    else if (power == 80)
    {
        return 7.0f;
    }
    else if (power == 100)
    {
        return 8.0f;
    }
    else
    {
        return 7.0f;
    }
}

/**
  * @brief          底盘无力
  * @param[in]      *Chassis_Stop_f：底盘主结构体
  * @retval         none
  */
void Chassis_Stop(chassis_control_t *Chassis_Stop_f)
{
    //*控制量
    Chassis_ch0 = Chassis_ch1 = Chassis_ch2 = 0.0f;
}

/**
  * @brief          返回底盘模式指针
  * @param[in]      none
  * @retval         
  */
const chassis_behaviour_e *get_chassis_behaviour_point(void)
{
    return (&chassis_behaviour);
}

/**
  * @brief          返回底盘模式
  * @param[in]      none
  * @retval         
  */
uint16_t re_chassis_behaviour(void)
{
    return (chassis_behaviour);
}


static void chassis_rescue(chassis_control_t *Rescue_Card ,int16_t dire)
{
    static uint8_t clock = 0 ;

    if(dire == 1 || dire == 2)
    {
        rescuemotorspeed_set_value = (dire * 2 - 3) * Rescue_Speed;

        if (int16_t_abs(Rescue_Card->rescue_motor_c->rescue_motor_measure->speed) <= 300)
        {
            clock++;

            if (clock > 5)
            {
                clock = 0;
                rescuemotorspeed_set_value = 0 ;
            }
        }
    }
    else
    {
        rescuemotorspeed_set_value = 0 ;
    }
}

static void rescue_clap(chassis_control_t *Rescue_Clap ,int16_t dire)
{
    static uint8_t clock = 0 ;

    if(dire == 1 || dire == 2)
    {
        rescueclapspeed_set_value = (dire * 2 - 3) * Rescue_Speed;

        if (int16_t_abs(Rescue_Clap->rescue_clap_c->rescue_clap_measure->speed) <= 300)
        {
            clock++;

            if (clock > 5)
            {
                clock = 0;
                rescueclapspeed_set_value = 0 ;
            }
        }
    }
    else
    {
        rescueclapspeed_set_value = 0 ;
    }
}

