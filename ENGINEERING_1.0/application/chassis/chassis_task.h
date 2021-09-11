/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       chassis_task.c/h
  * @brief      完成底盘控制任务
  * @note       合并版本
  * @history    v5.0
  *
  @verbatim   
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
  */
#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__
#include "struct_typedef.h"
#include "pid.h"
#include "imu.h"
#include "maths.h"
#include "rc.h"
#include "can_1_receive.h"
#include "can_2_receive.h"
#include "shoot_task.h"
#include "parameter.h"

/*OS控制任务周期以及启动时间*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME   2

//底盘测试模式 宏定义 0 为不使用测试模式
#define CHASSIS_TEST_MODE   0

/**********************底盘电机pid参数**********************/
#define CHASSIS_MOTOR1_PID_Kp    9.0f
#define CHASSIS_MOTOR1_PID_Ki    0.0f
#define CHASSIS_MOTOR1_PID_Kd    0.5f

#define CHASSIS_MOTOR2_PID_Kp    9.0f //8
#define CHASSIS_MOTOR2_PID_Ki    0.0f
#define CHASSIS_MOTOR2_PID_Kd    0.5f

#define CHASSIS_MOTOR3_PID_Kp    9.0f
#define CHASSIS_MOTOR3_PID_Ki    0.0f
#define CHASSIS_MOTOR3_PID_Kd    0.5f

#define CHASSIS_MOTOR4_PID_Kp    9.0f //8
#define CHASSIS_MOTOR4_PID_Ki    0.0f
#define CHASSIS_MOTOR4_PID_Kd    0.5f

#define CHASSIS_LOCATION_PID_P  100.0f  //位置,ctrl
#define CHASSIS_LOCATION_PID_I  0.0f
#define CHASSIS_LOCATION_PID_D  10.8f

#define CHASSIS_ROTATE_FOLLOW_P  7.0f  //PID   8.0  13
#define CHASSIS_ROTATE_FOLLOW_I  0.01f   //0.01
#define CHASSIS_ROTATE_FOLLOW_D  20.0f   //5.02   10.02   2

#define SGIMBAL_YAW_P_P	60.0f
#define	SGIMBAL_YAW_P_I	0.0f
#define	SGIMBAL_YAW_P_D	0.0f

#define	SGIMBAL_YAW_S_P	3.0f
#define	SGIMBAL_YAW_S_I	0.0f
#define	SGIMBAL_YAW_S_D	0.01f

#define	SGIMBAL_PITCH_P_P	20.0f
#define	SGIMBAL_PITCH_P_I	0.0f
#define	SGIMBAL_PITCH_P_D	0.0f

#define	SGIMBAL_PITCH_S_P	4.0f
#define	SGIMBAL_PITCH_S_I	0.0f
#define	SGIMBAL_PITCH_S_D	0.0f

#define  RESCUE_S_P   20.0f
#define  RESCUE_S_I   0.0f
#define  RESCUE_S_D   1.0f

#define  RESCUECLAP_S_P   20.0f
#define  RESCUECLAP_S_I   0.0f
#define  RESCUECLAP_S_D   1.0f



/**********************低通滤波比例**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K  0.0410f  //0.0110f 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高  |  0.26f  |  0.0097f  |  0.0510f
#define CHASSIS_MK_SECOND_FILTERING   (0.8f)

/**********************运动加速度限制**********************/
#define STRAIGHT_ACCELERAD        3.5f      //直行底盘加速度限制
#define TRANSLATION_ACCELERAD     5.5f      //平移底盘加速度限制
#define ROTATING_ACCELERAD        19.0f     //旋转底盘加速度限制


#define CHASSIS_ROTATION_SPEED_1    (600)
#define CHASSIS_ROTATION_SPEED_2    (750)
#define CHASSIS_ROTATION_SPEED_3    (1200)  //900

#define CHASSIS_ROTATION_SPEED        100      //小陀螺的旋转速度  2000
#define CHASSIS_ROTATION_MOVE_SPEED  (800*0.8f)   //小陀螺移动时为防止轨迹失真减转速   1700
#define CHASSIS_TWIST_SPEED           600      //扭腰速度  1600

#define NORMAL_FORWARD_BACK_SPEED 	660.0f   //键盘普通直行速度
#define NORMAL_LEFT_RIGHT_SPEED   	660.0f   //键盘普通平移速度

#define HIGH_FORWARD_BACK_SPEED 	750.0f     //键盘加速直行速度  600.0f 
#define HIGH_LEFT_RIGHT_SPEED   	750.0f     //键盘加速平移速度

/*底盘电机数据*/
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
	
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
	fp32 position;

	int16_t give_current;
    int16_t pid_output;
} motor_t;

/*键盘按键结构体*/
typedef struct
{
	float key_w;
    float key_s;
    float key_a;
    float key_d;
	
		float mousex;
	
		float press;
	
		float mousey;
	
	float last_key_w;
    float last_key_s;
    float last_key_a;
    float last_key_d;
		float last_mousex;
} key_wsad_t;


typedef struct
{
    const rc_ctrl_t *chassis_RC;             //底盘使用的遥控器指针
		const motor_measure_t *rescue_motor_measure;
		key_wsad_t key;
		int16_t rescue_motor_output;
	
    fp32 accel;
    fp32 speed;
    fp32 speed_set_value;
	fp32 position;
	pid_parameter_t rescue_motor_pid;	

	
	pid_parameter_t rescue_motor_pid_p;
	pid_parameter_t rescue_motor_pid_s;

	

} rescue_motor_control_t;


typedef struct
{
    const rc_ctrl_t *chassis_RC;             //底盘使用的遥控器指针
		const motor_measure_t *rescue_clap_measure;
	
		key_wsad_t key;
		int16_t rescue_clap_output;
	
    fp32 accel;
    fp32 speed;
    fp32 speed_set_value;
		fp32 position;

	pid_parameter_t rescue_motor_pid;
	pid_parameter_t rescue_clap_pid;
	

	pid_parameter_t rescue_clap_pid_p;
	pid_parameter_t rescue_clap_pid_s;
	

} rescue_clap_control_t;

typedef struct
{
    const rc_ctrl_t *chassis_RC;             //底盘使用的遥控器指针
		const motor_measure_t *yaw6020_measure;
	
		key_wsad_t key;

    fp32 accel;
    fp32 speed;
    fp32 speed_set;
	fp32 position;
	
		int16_t yaw_6020_output;
	
	pid_parameter_t yaw_6020_pid_p;
	pid_parameter_t yaw_6020_pid_s;

	

} syaw_control_t;

typedef struct
{
    const rc_ctrl_t *chassis_RC;             //底盘使用的遥控器指针
		const motor_measure_t *pitch2006_measure;
	
	key_wsad_t key;
	
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
		fp32 position;
	
		int16_t pitch_2006_output;
	

	pid_parameter_t pitch_2006_pid_p;
	pid_parameter_t pitch_2006_pid_s;
	

	

} spitch_control_t;


/*底盘整体数据结构体*/
typedef struct
{
    const rc_ctrl_t *chassis_RC;             //底盘使用的遥控器指针
    const gimbal_yaw_receive_t *gimbal_re_data; //云台板处理数据，数据由can2传输给底盘板，底盘板再输出给yaw轴电机
	fire_task_t *fire_c;
	key_wsad_t key;
	//motor_measure_t *yaw_motor_measure;      //can1直接接收的yaw轴数据
	const Supercapacitor_receive_t *super_cap_c;
	
    motor_t chassis_motor[4];        //底盘电机数据(包含电机统一结构体指针)	
	
	rescue_motor_control_t *rescue_motor_c;
	rescue_clap_control_t *rescue_clap_c;
	
	syaw_control_t *yaw_6020_c;
	spitch_control_t *pitch_2006_c;
	
		PYR_t *gyro;
	
    pid_parameter_t chassis_speed_pid[4]; //正常移动pid
	pid_parameter_t chassis_location_pid; //底盘位置环pid
    pid_parameter_t chassis_rotate_pid;   //旋转pid
	pid_parameter_t chassis_power_pid;   //功率限制pid
	
    first_order_filter_type_t LowFilt_chassis_vx; //低通滤波器
    first_order_filter_type_t LowFilt_chassis_vy; //低通滤波器
	
    fp32 speed_x_set; //底盘设定速度 前进方向 前为正，单位 m/s
    fp32 speed_y_set; //底盘设定速度 左右方向 左为正，单位 m/s
    fp32 speed_z_set; //底盘设定旋转角速度，逆时针为正 单位 rad/s

    fp32 chassis_gimbal_angel; //底盘与云台的角度
	
	uint8_t SuperCap_discharge_flag;  //超级电容放电标志位
	
	fp32 chassis_speed_gain;
	fp32 chassis_last_speed_gain;
	
	u8 chassis_re_sensorL;
	u8 chassis_re_sensorR;
	//uint8_t Chassis_keyboad;
} chassis_control_t;


//底盘主任务
extern void chassis_task(void *pvParameters);
//底盘控制量设置
extern void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2);

extern void sgimbal_set_remote(chassis_control_t *sgimabl_set_f, int16_t ch0, int16_t ch1);

extern void rescue_set_remote(chassis_control_t *rescue_set_f, int16_t rescuespeed_set ,int16_t resclapspeed_set);

extern void chassis_task_off(u8 options);

extern chassis_control_t *get_chassis_control_point(void);

#endif
