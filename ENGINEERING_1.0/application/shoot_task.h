#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__
#include "struct_typedef.h"
#include "pid.h"
#include "rc.h"
#include "can_1_receive.h"
#include "freertos.h"
#include "task.h"


/*OS控制任务周期以及启动时间*/
#define FIRE_TASK_INIT_TIME  5
#define FIRE_CONTROL_TIME_MS 2

/*摩擦轮*/
#define PWM_Shoot_Left   TIM4->CCR1  //PD12
#define PWM_Shoot_Right  TIM4->CCR2	 //PD13

/**********拨弹电机速度环pid参数定义**********/
#define FIRE_S_P 15.0f   //供弹电机M2006速度环
#define FIRE_S_I 0.2f
#define FIRE_S_D 2.0f

/**********发弹系统速度设定**********/
//零级：15m/s   一级：18m/s  二级：22m/s  三级：30m/s
#define FRICTION_THREE_SHOOT_SPEED     1400       //30摩擦轮高速pwm
#define FRICTION_TWO_SHOOT_SPEED       1363       //22摩擦轮高速pwm
#define FRICTION_ONE_SHOOT_SPEED       1322       //18摩擦轮高速pwm
#define FRICTION_ZERO_SHOOT_SPEED      1305       //15摩擦轮低速pwm    
#define FRICTION_SHOOT_STOP            1000       //0摩擦轮停止pwm
#define LOADING_SPEED_H                (8000)     //供弹电机速度 3000    2500
#define LOADING_SPEED_L                (4500)     //供弹电机速度 3000    2500

//卡弹时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         10.0f
#define BLOCK_TIME                  500  //700
#define REVERSE_TIME                500


/*发弹模式*/
typedef enum
{
    FIRE_H = 0,    //高射频
	FIRE_L,        //低射频
    AUTO_FIRE,     //自动发射
    STOP_FIRE,     //停止发射
    BACK,          //退弹
	FIRE_ERROR,
} fire_work_status_e;

/*摩擦轮模式*/
typedef enum
{
    LOW_SPEED,
    HIGH_SPEED,
    STOP_SHOOT,
	SHOOT_ERROR,
} shoot_work_status_e;

/*火控结构体*/
typedef struct Fire_param
{
    const motor_measure_t *fire_motor_measure;
    const rc_ctrl_t *fire_RC;   //开火任务使用的遥控器指针
    pid_parameter_t fire_s_pid;      //拨弹2006电机pid

    fire_work_status_e fire_mode;   //射弹模式
	fire_work_status_e last_fire_mode;   //射弹模式
    shoot_work_status_e friction_mode;  //摩擦轮模式

	uint8_t fire_switch;    //拨弹开关
    int16_t GD_output;      //供弹电机输出
	int16_t GD_set_speed;   //供弹电机速度
    int shoot_speed;        //当前设置射速
    //uint8_t dead_mode;      //死亡模式开关
	
	uint16_t hot_max;      //最大热量
	uint16_t hot_current;  //当前热量
	
	fp32 GD_speed_gain;   //用于热量限制
	
    uint16_t block_time;    //堵转时间
    uint16_t reverse_time;  //回转时间

} fire_task_t;


TaskHandle_t *get_shoot_task_handle(void);
extern fire_task_t *get_fire_control_point(void);
extern void shoot_task(void *pvparameters);
extern void fire_task_off(void);
extern void shoot_hot_limit(void);

void shoot_app_init(void);
uint16_t re_shoot_status(void);

#endif // __SHOOT_TASK_H__
