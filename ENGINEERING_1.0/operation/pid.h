#ifndef __PID_H
#define __PID_H
#include "struct_typedef.h"


typedef enum
{
	PID_NORMAL = 0,  //正常
	CHANGE_INTEGRAL, //变积分
	STOP_SATURATED, //抗饱和
	
	PID_MANUAL, //手动模式
} pid_mode_e;

typedef struct //pid结构体变量
{
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 SetValue;
    fp32 ActualValue;

    fp32 out;
	fp32 lastout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	
	fp32 Ierror;
    fp32 Derror[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3];   //误差项 0最新 1上一次 2上上次
	
	fp32 stepIn;
	
	/* 误差死区 */
	fp32 deadband;
	
	/* 积分分离 */
    fp32 epsilon; //积分分离阈(yu)值
	
	/* 不完全微分 */
	fp32 alpha;

    /* 抗积分饱和 */
    fp32 maximum; //最大值
    fp32 minimum; //最小值

    /* 变积分 */
    fp32 errorabsmax; //偏差绝对值最大值
    fp32 errorabsmin; //偏差绝对值最小值

	/* 微分先行 */
    fp32 gama; //微分先行滤波系数
	
	pid_mode_e pid_mode;
} pid_parameter_t;


extern int32_t location_pid_int32(pid_parameter_t *pid , float actualValue);
extern void pid_init(pid_parameter_t *pid,fp32 kp,fp32 ki,fp32 kd,fp32 i_max,fp32 out_max);
extern void pid_clear(pid_parameter_t *pid);
extern float increment_pid(pid_parameter_t *pid , float actualValue);
/*步进式PID控制设定值步进处理函数*/
float step_in_processing(pid_parameter_t *vPID, float sp);
int32_t pid_regulator(pid_parameter_t *vPID , float actualValue);

#endif















