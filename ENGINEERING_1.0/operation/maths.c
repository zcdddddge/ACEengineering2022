#include "maths.h"
#include "list_of_function.h"
#include <stddef.h>
#include "arm_math.h"
#include "parameter.h"

//数值限制
int32_t int32_limit(int32_t x, int32_t max, int32_t min)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

int16_t int16_limit(int16_t x, int16_t max, int16_t min)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

float float_limit(float x, float max, float min)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

signed long limit_long(signed long x, signed long max, signed long min)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

int16_t int16_t_abs(int16_t x) // 绝对值计算
{
    if (x < 0)
        x = -x;
    return x;
}

signed long long_abs(signed long x) // 绝对值计算
{
    if (x < 0)
        x = -x;
    return x;
}

float float_abs(float x) // 绝对值计算
{
    if (x < 0)
        x = -x;
    return x;
}

float invSqrt(float x) //平方根倒数速算法
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*取两个数最大值的绝对值*/
int16_t max_abs(int16_t x, int16_t y)
{
    if (int16_t_abs(x) >= int16_t_abs(y))
        return int16_t_abs(x);
    else
        return int16_t_abs(y);
}

/*
*功能：运动控制斜坡函数（加速度限制）
*传入：1.加速度限制对应结构体  2.控制量 3.限制加速度
*传出：处理量
*类型：16位整形
*/
int16_t motion_acceleration_control(acceleration_control_type_t *acceleration_control, int16_t Input, int16_t Limit)
{
    acceleration_control->Input = Input;
    acceleration_control->acc_limit = Limit;

    acceleration_control->acc_now = acceleration_control->Input - acceleration_control->Last_Input;

    if (int16_t_abs(acceleration_control->acc_now) > acceleration_control->acc_limit)
    {
        acceleration_control->Output = acceleration_control->Last_Input + acceleration_control->acc_now / int16_t_abs(acceleration_control->acc_now) * acceleration_control->acc_limit;
    }

    acceleration_control->Last_Input = acceleration_control->Output;

    return acceleration_control->Output;
}

//一阶低通滤波计算
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
    first_order_filter_type->last_input = first_order_filter_type->out;

    return first_order_filter_type->out;
}

//一阶低通滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num)
{
    if (first_order_filter_type == NULL)
    {
        return;
    }

    first_order_filter_type->input = 0;
    first_order_filter_type->last_input = 0;
    first_order_filter_type->num = num;
    first_order_filter_type->out = 0;
}

/*
*功能：滑动均值滤波参数初始化(浮点型)
*输入：滤波对象结构体
*/
void sliding_mean_filter_init(sliding_mean_filter_type_t *mean_filter)
{
    mean_filter->count_num = 0;
    for (int i = 0; i < 20; i++)
        mean_filter->FIFO[i] = 0.0f;
    mean_filter->Input = 0.0f;
    mean_filter->Output = 0.0f;
    mean_filter->Sum = 0.0f;
    mean_filter->sum_flag = 0;
}

/*
*功能：滑动均值滤波（浮点型）------抑制小幅度高频噪声
*传入：1.滤波对象结构体  2.更新值 3.均值数量
*传出：滑动滤波输出值（250次）
*/
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, float Input, int num)
{
    //更新
    mean_filter->Input = Input;
    mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;
    mean_filter->count_num++;

    if (mean_filter->count_num == num)
    {
        mean_filter->count_num = 0;
        mean_filter->sum_flag = 1;
    }
    //求和
    if (mean_filter->sum_flag == 1)
    {
        for (int count = 0; count < num; count++)
        {
            mean_filter->Sum += mean_filter->FIFO[count];
        }
    }
    //均值
    mean_filter->Output = mean_filter->Sum / num;
    mean_filter->Sum = 0;

    return mean_filter->Output;
}

/*
*功能：正负循环限制（16位）
*传入：1.输入值  2.限制幅度(正数)  
*传出：限幅输出值
*描述：将输入值限制在 +-限制幅度 的范围内
*/
int16_t loop_restriction_int16(int16_t num, int16_t limit_num)
{
    if (int16_t_abs(num) > limit_num)
    {
        if (num >= 0)
            num -= limit_num;
        else
            num += limit_num;
    }
    return num;
}

/*
*功能：正负循环限制（float）
*传入：1.输入值  2.限制幅度(正数)  
*传出：限幅输出值
*描述：将输入值限制在 +-限制幅度 的范围内
*/
float loop_restriction_float(float num, float limit_num)
{
    if (float_abs(num) > limit_num)
    {
        if (num >= 0)
            num -= limit_num;
        else
            num += limit_num;
    }
    return num;
}

/*循环限幅32*/
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//斜坡函数(加速度限制)
void data_accelerated_control(float *input, float acc)
{
    static int16_t last_num = 0;
    int16_t temp;
    temp = *input - last_num;

    if (float_abs(temp) > acc)
        *input = last_num + temp / float_abs(temp) * acc;

    last_num = *input;
}



#define trigonometric_function 2


//sin函数查表法 输入角度制
float sin_calculate(float angle)
{
#if (trigonometric_function == 1)
    float sin_angle;

    if (angle >= 0.0f && angle < 90.0f)
        sin_angle = (Trigonometric_Functions[(int)(float_abs(angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        sin_angle = (Trigonometric_Functions[(int)(float_abs(180.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= -180.0f && angle < -90.f)
        sin_angle = -(Trigonometric_Functions[(int)(float_abs(180.0f + angle) * 10.0f)] / 100.0f);
    else if (angle >= -90.0f && angle < 0.f)
        sin_angle = -(Trigonometric_Functions[(int)(float_abs(180.0f - (180.0f + angle)) * 10.0f)] / 100.0f);
	else if (angle == 180.f)
		sin_angle = 0.0f;

    return sin_angle;
#elif (trigonometric_function == 2)
	return arm_sin_f32(angle / RAD2DEG); //弧度制
#endif
}

//cos函数查表法 输入角度制
float cos_calculate(float angle)
{
#if (trigonometric_function == 1)
    float cos_angle;

    angle = float_abs(angle);

    if (angle >= 0.0f && angle < 90.0f)
        cos_angle = (Trigonometric_Functions[(int)(float_abs(90.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        cos_angle = -(Trigonometric_Functions[(int)(float_abs(angle - 90.0f) * 10.0f)] / 100.0f);
	else if (angle == 180.f)
		cos_angle = -1.0f;

    return cos_angle;
#elif (trigonometric_function == 2)
	return arm_cos_f32(angle / RAD2DEG); //弧度制
#endif
}


