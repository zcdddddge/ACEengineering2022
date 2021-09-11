#include "pid.h"     
#include "maths.h"
#include <stddef.h>
#include <math.h>

#pragma diag_suppress 177

static fp32 variable_integral_coefficient(fp32 error, fp32 absmax, fp32 absmin); //变积分函数
static uint16_t beta_generation(float error, float epsilon); //积分分离函数


int32_t pid_chassis_follow(pid_parameter_t *pid , float actualValue) 
{		
	//error
	pid->error[0] = pid->SetValue - actualValue;      

	if (fabs(pid->error[0]) > pid->deadband)
	{
		//误差积分	
		//pid->Ierror += pid->error[0];
		pid->Ierror += (pid->error[0] + pid->error[1]) / 2;
		pid->Ierror *= variable_integral_coefficient(pid->error[0], pid->errorabsmax, pid->errorabsmin);
		//积分限幅
		pid->Ierror = int32_limit(pid->Ierror, 10000, -10000);
		
		//微分项 记录
		//pid->Derror[2] = pid->Derror[1];
		//pid->Derror[1] = pid->Derror[0];
		pid->Derror[0] = (pid->error[0] - pid->error[1]);
		
		//输出pid运算
		pid->out = (pid->Kp * pid->error[0]) + (pid->Ki * pid->Ierror) + (pid->Kd * pid->Derror[0]);  
	}
	else
	{
		pid_clear(pid);
	}
	
	//PID每一项输出
	pid->Pout = (pid->Kp * pid->error[0]);
	pid->Iout = (pid->Ki * pid->Ierror);
	pid->Dout = (pid->Kd * pid->Derror[0]);
	
	//记录error
	pid->error[1] = pid->error[0];   
	
	return pid->out;
}


/*
*功能：对传入数据进行位置式pid运算并输出值
*传入：1.PID结构体名称   2.当前值
*传出：对该设备的控制量
*说明：目标值在闭环控制函数里赋值
*/
int32_t location_pid_int32(pid_parameter_t *pid , float actualValue) 
{
	if (pid->pid_mode == PID_MANUAL)
	{
		pid_clear(pid);
		return 0;
	}
		
	//error
	pid->error[0] = pid->SetValue - actualValue;      
	
	//误差积分	
	//pid->Ierror += pid->error[0];
	pid->Ierror += (pid->error[0] + pid->error[1]) / 2;

	//积分限幅
	pid->Ierror = int32_limit(pid->Ierror, 5000, -5000);
	
	//微分项 记录
	//pid->Derror[2] = pid->Derror[1];
	//pid->Derror[1] = pid->Derror[0];
	pid->Derror[0] = (pid->error[0] - pid->error[1]);
	
	//输出pid运算
	pid->out = (pid->Kp * pid->error[0]) + (pid->Ki * pid->Ierror) + (pid->Kd * pid->Derror[0]);   
	
	//PID每一项输出
	pid->Pout = (pid->Kp * pid->error[0]);
	pid->Iout = (pid->Ki * pid->Ierror);
	pid->Dout = (pid->Kd * pid->Derror[0]);
	
	//记录error
	pid->error[1] = pid->error[0];   
	
	return pid->out;
}



//底盘pid初始化
void pid_init(pid_parameter_t *pid, fp32 kp, fp32 ki, fp32 kd, fp32 i_max, fp32 out_max)
{
	if (pid == NULL)
	{
		return;
	}

	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

    pid->SetValue = 0;
    pid->ActualValue = 0;

    pid->out = 0;
	pid->lastout = 0;
	
	pid->Ierror = 0;
	
//	pid->max_iout = i_max;
//	pid->max_out = out_max;
	
	pid->Derror[0] = pid->Derror[1] = pid->Derror[2] = 0.0f;
	pid->error[0] = pid->error[0] = pid->error[0] = 0.0f;
	pid->Dout = pid->Iout = pid->Pout = pid->out = 0.0f;
}



void pid_clear(pid_parameter_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Derror[0] = pid->Derror[1] = pid->Derror[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->ActualValue = pid->SetValue = 0.0f;
}


//增量式PID
float increment_pid(pid_parameter_t *pid , float actualValue)
{
	//error
	pid->error[0] = pid->SetValue - actualValue;
	
	//微分项 记录
	pid->Derror[0] = pid->error[0] - 2 * pid->error[1] + pid->error[2];
	
	//pid->Ierror = (pid->error[0] - pid->error[1]) / 2;
	pid->Ierror = pid->error[0];
	pid->Ierror *= variable_integral_coefficient(pid->error[0], pid->errorabsmax, pid->errorabsmin);
	
	//输出pid运算
	pid->out = (pid->Kp * (pid->error[0] - pid->error[1])) + (pid->Ki * pid->Ierror) + (pid->Kd * pid->Derror[0]);   
	
	//PID每一项输出
	pid->Pout = (pid->Kp * (pid->error[0] - pid->error[1]));
	pid->Iout = (pid->Ki * pid->Ierror);
	pid->Dout = (pid->Kd * pid->Derror[0]);
	
	//记录error
	pid->error[2] = pid->error[1]; //记录上上次误差
	pid->error[1] = pid->error[0]; //记录上次误差
	
	return pid->out;
}




///**
//  * @brief          抗积分饱和PID控制器
//  * @param[in]      pid: pid结构体
//  * @param[in]      actualValue: 实际值
//  * @retval         对该设备的控制量
//  * @attention      要用到 maximum 和 minimum 
//  */
//int32_t pid_antiintegral_saturation(pid_parameter_t *vPID, fp32 actualValue)
//{
//    int32_t dError;

//    //error
//    vPID->error[0] = vPID->SetValue - actualValue;

//    //误差积分
//    if (vPID->out > vPID->maximum)
//    {
//        if (vPID->error[0] <= 0)
//        {
//            vPID->Ierror += vPID->error[0];
//        }
//    }
//    else if (vPID->out < vPID->minimum)
//    {
//        if (vPID->error[0] >= 0)
//        {
//            vPID->Ierror += vPID->error[0];
//        }
//    }
//    else
//    {
//        vPID->Ierror += vPID->error[0];
//    }
//    //积分限幅
//    vPID->Ierror = int32_limit(vPID->Ierror, vPID->max_iout, -vPID->max_iout);

//    //微分
//    dError = vPID->error[0] - vPID->error[1];

//    //微分项 记录
//    vPID->Derror[2] = vPID->Derror[1];
//    vPID->Derror[1] = vPID->Derror[0];
//    vPID->Derror[0] = (vPID->error[0] - vPID->error[1]);

//    vPID->out = (vPID->Kp * vPID->error[0]) + (vPID->Ki * vPID->Ierror) + (vPID->Kd * dError);

//    //PID每一项输出
//    vPID->Pout = (vPID->Kp * vPID->error[0]);
//    vPID->Iout = (vPID->Ki * vPID->Ierror);
//    vPID->Dout = (vPID->Kd * dError);

//    //记录error
//    vPID->error[1] = vPID->error[0];

//    return vPID->out;
//}

/**
  * @brief          变积分系数处理函数，实现一个输出0和1之间的分段线性函数
  * @param[in]      error: 当前输入的偏差值
  * @param[in]      absmax: 偏差绝对值的最大值
  * @param[in]      absmin: 偏差绝对值的最小值
  * @retval         factor: 控制积分的比例
  * @attention      当偏差的绝对值小于最小值时，输出为1；当偏差的绝对值大于最大值时，输出为0
  *                 当偏差的绝对值介于最大值和最小值之间时，输出在0和1之间现行变化
  */
static fp32 variable_integral_coefficient(fp32 error, fp32 absmax, fp32 absmin)
{
    fp32 factor = 0.0f;

    if (float_abs(error) <= absmin) //最小值
    {
        factor = 1.0f;
    }
    else if (float_abs(error) > absmax) //最大值
    {
        factor = 0.0f;
    }
    else
    {
        factor = (absmax - float_abs(error)) / (absmax - absmin);
    }

    return factor;
}

/**
  * @brief          积分分离函数
  * @param[in]      error: 当前的误差值
  * @param[in]      epsilon: 误差阈值
  * @retval         1: 加上积分项
  *                 0: 去除积分项
  * @attention      beta = beta_generation(error, vPID->epsilon);
  */
static uint16_t beta_generation(float error, float epsilon)
{
	uint16_t beta = 0;

	if(float_abs(error) <= epsilon)
	{
		beta = 1;
	}

	return beta;

}

///**
//  * @brief          微分先行PID
//  * @param[in]      pid: pid结构体
//  * @param[in]      actualValue: 实际值
//  * @retval         对该设备的控制量
//  * @attention      在某些给定值频繁且大幅变化的场合，微分项常常会引起系统的振荡。为了适应这种给定值频繁变化的场合，人们设计了微分先行算法。
//  *                 要用到 gama 
//  */
//int32_t PIDRegulation(pid_parameter_t *vPID, fp32 actualValue)
//{
//    fp32 c1, c2, c3, temp;

//    //error
//    vPID->error[0] = vPID->SetValue - actualValue;

//    /******************误差积分******************/
//    /* 正常积分 */
//    //vPID->Ierror += vPID->error[0];  //正常积分
//    /* 梯形积分 */
//    vPID->Ierror += (vPID->error[0] + vPID->error[1]) / 2;
//    /* 变积分PID */
//    //fp32 x = variable_integral_coefficient(vPID->error[0], vPID->errorabsmax, vPID->errorabsmin);
//    //vPID->Ierror += x * (vPID->error[0] + vPID->error[1]) / 2; //梯形积分

//    //积分限幅
//    vPID->Ierror = int32_limit(vPID->Ierror, vPID->max_iout, -vPID->max_iout);

//    //微分项 记录
//    temp = vPID->gama * vPID->Kd + vPID->Kp;
//    c3 = vPID->Kd / temp;
//    c2 = (vPID->Kd + vPID->Kp) / temp;
//    c1 = vPID->gama * c3;
//    vPID->Derror[0] = c1 * vPID->Derror[0] + c2 * actualValue + c3 * vPID->Last_ActualValue;

//    //输出pid运算
//    vPID->out = (vPID->Kp * vPID->error[0]) + (vPID->Ki * vPID->Ierror) + (vPID->Kd * vPID->Derror[0]);

//    //PID每一项输出
//    vPID->Pout = (vPID->Kp * vPID->error[0]);
//    vPID->Iout = (vPID->Ki * vPID->Ierror);
//    vPID->Dout = (vPID->Kd * vPID->Derror[0]);

//    //记录error
//    vPID->error[1] = vPID->error[0];
//    //记录实际值
//    vPID->Last_ActualValue = actualValue;

//    return vPID->out;
//}

///**
//  * @brief          实现前馈控制器
//  * @param[in]      vFFC: FFC结构体
//  * @retval         对该设备的控制量
//  * @attention      有了这个前馈控制器，只需要与PID控制器的输出合并在一起就好了U(k)=Up(k)+Uf(k)
//  *                 使用前: vFFC->rin=vPID-> setpoint
//  *                 要用到 Alpha 和 Beta
//  */
//fp32 FeedforwardController(FFC *vFFC)
//{
//    fp32 result;

//    result = vFFC->Alpha * (vFFC->rin - vFFC->lastRin) + vFFC->Beta * (vFFC->rin - 2 * vFFC->lastRin + vFFC->perrRin);

//    vFFC->perrRin = vFFC->lastRin;

//    vFFC->lastRin = vFFC->rin;

//    return result;
//}



/*步进式PID控制设定值步进处理函数*/
float step_in_processing(pid_parameter_t *vPID, float sp)
{
    //float stepIn = (vPID->maximum - vPID->minimum) * 0.1f + vPID->minimum;
	float stepIn = vPID->stepIn;
    float kFactor = 0.0f;

    if (fabs(vPID->SetValue - sp) <= stepIn)
    {
        vPID->SetValue = sp;
    }
    else
    {
        if (vPID->SetValue - sp > 0)
        {
            kFactor = -1.0f;
        }
        else if (vPID->SetValue - sp < 0)
        {
            kFactor = 1.0f;
        }
        else
        {
            kFactor = 0.0f;
        }

        vPID->SetValue = vPID->SetValue + kFactor * stepIn;
    }

    return vPID->SetValue;
}


// NO_4 修改增加
int32_t pid_regulator(pid_parameter_t *vPID , float actualValue)
{
	//error
	vPID->error[0] = vPID->SetValue - actualValue;   

//	if (fabs(vPID->error[0]) <= vPID->deadband)
//	{
//		vPID->error[0] = 0.0f;
//	}	
	
	//误差积分	
//	if (beta_generation(vPID->error[0], 5.0f) == 1)
//	{
//		
//	}
	//vPID->Ierror += (vPID->error[0] + vPID->error[1]) / 2; //梯形积分
	
//	//maximum和I  
//	if (vPID->out > vPID->maximum) //vPID->maximum
//	{
//		if (vPID->error[0] <= 0)
//		{
//			vPID->Ierror += vPID->error[0];
//		}
//	}
//	else if (vPID->out < vPID->minimum) //vPID->minimum
//	{
//		if (vPID->error[0] >= 0)
//		{
//			vPID->Ierror += vPID->error[0];
//		}
//	}
//	else
//	{
//		vPID->Ierror += vPID->error[0];
//	}
	
	//变积分 TODO
	vPID->Ierror += (vPID->error[0] + vPID->error[1]) / 2;
	vPID->Ierror *= variable_integral_coefficient(vPID->error[0], vPID->errorabsmax, vPID->errorabsmin);
	
	//积分限幅
	vPID->Ierror = int32_limit(vPID->Ierror, vPID->max_iout, -vPID->max_iout);
	
	//微分项 记录
	//vPID->Derror[2] = vPID->Derror[1];
	//vPID->Derror[1] = vPID->Derror[0];
	vPID->Derror[0] = (vPID->error[0] - vPID->error[1]);
	
	//PID每一项输出
	vPID->Pout = (vPID->Kp * vPID->error[0]);
	vPID->Iout = (vPID->Ki * vPID->Ierror);
	vPID->Dout = (vPID->Kd * vPID->Derror[0]);
	//计算微分项增量带不完全微分
	//vPID->Dout = (vPID->Kd * vPID->Derror[0] * (1 - vPID->alpha)) + (vPID->alpha * vPID->Dout);
	
//	if (fabs(vPID->error[0]) <= vPID->deadband)
//	{
//		vPID->error[0] = vPID->error[1] = vPID->error[2] = 0.0f;
//		vPID->Derror[0] = vPID->Derror[1] = vPID->Derror[2] = 0.0f;
//		vPID->out = vPID->Pout = vPID->Iout = vPID->Dout = 0.0f;
//	}
	
	//输出PID运算
	vPID->out = vPID->Pout + vPID->Iout + vPID->Dout;   
	
	//记录error
	vPID->error[1] = vPID->error[0];
	
	return vPID->out;
}








