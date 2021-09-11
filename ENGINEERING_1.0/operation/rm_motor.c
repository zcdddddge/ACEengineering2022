#include "rm_motor.h"
#include <stddef.h>
#include "parameter.h"

/**
 *文件内容： 1.各种电机的PID结构体定义和电机真实值的定义
 *          2.电机的闭环算法（速度环，速度位置环）       
 *          3.计算减速电机码盘值的算法
 *          4.电机码盘临界处理
 *          5.加速度限制算法
 *          6.电机堵转检测
 *          7.计算云台底盘差角算法
 *备注：结构体赋值初始化在system文件里
 *      PID参数定义在PID_Def文件里
 */

/*=========================电机闭环算法==============================*/
/*
功能：速度环控制；
传入参数：速度环pid结构体spid，目标速度值setSpeed，实际速度值actualSpeed
传出：电流
*/
int16_t motor_speed_control(pid_parameter_t *spid, int16_t setSpeed, int16_t actualSpeed, int16_t current_limit)
{
    spid->SetValue = setSpeed;
    spid->ActualValue = actualSpeed;

    location_pid_int32(spid, spid->ActualValue);
    spid->out = int32_limit(spid->out, current_limit, -current_limit);

    return spid->out;
}

/*
*功能：位置速度串级闭环控制
*传入：电机速度pid结构体，电机位置pid结构体，电机真实位置，电机真实速度，位置目标值，电流限制值
*传出：电流
*/
int16_t motor_position_speed_control(pid_parameter_t *speed_pid, pid_parameter_t *position_pid, int16_t actual_position, int16_t actual_speed, int16_t setPosition, int16_t current_limit)
{
    int32_t output;

    position_pid->SetValue = setPosition;                                    //位置环设定值为输入控制量 陀螺仪角度处理 (保持头始终向前)
    speed_pid->SetValue = location_pid_int32(position_pid, actual_position); //速度环设定值由位置环处理

    output = location_pid_int32(speed_pid, actual_speed);        //电机输出量
    output = int32_limit(output, current_limit, -current_limit); //输出限制

    return output;
}

int16_t motor_position_Stepping(pid_parameter_t *speed_pid, pid_parameter_t *position_pid, int16_t actual_position, int16_t actual_speed, int16_t setPosition, int16_t current_limit)
{
    int32_t output;

    //position_pid->SetValue = setPosition;   //位置环设定值为输入控制量 陀螺仪角度处理 (保持头始终向前)
    step_in_processing(position_pid, setPosition); //步进PID 设定值处理

    speed_pid->SetValue = pid_regulator(position_pid, actual_position); //速度环设定值由位置环处理

    output = location_pid_int32(speed_pid, actual_speed);        //电机输出量
    output = int32_limit(output, current_limit, -current_limit); //输出限制

    return output;
}

/*=========================RM电机一般算法========================*/
/*真实码盘值计算(通用，无论是什么电机或编码器，只要涉及到单圈增量式的都可以用)
*传入：电机数据结构体，减速比乘传动比，单圈码盘值
*传出：初始值为0，左正右负，半圈过渡的数据
*/
void motor_actual_position(motor_measure_t *rmMotor, int16_t gear_Ratio, int16_t lap_encoder)
{
    if (rmMotor->first_Flag == 0) //第一次进入时记录码盘值
    {
        rmMotor->last_position = rmMotor->position;
        rmMotor->first_Flag = 1;
    }
    rmMotor->actual_Position += angle_limiting_int16(rmMotor->position - rmMotor->last_position, lap_encoder); //差值累加
    rmMotor->actual_Position = check_codevalue(rmMotor->actual_Position, gear_Ratio, lap_encoder);             //过临界值复位码盘值
    rmMotor->last_position = rmMotor->position;
}

//临角处理16位（对应角度正值）
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder)
{
    if (Angl_Err < -(lap_encoder / 2))
    {
        Angl_Err += (lap_encoder - 1);
    }
    if (Angl_Err > (lap_encoder / 2))
    {
        Angl_Err -= (lap_encoder - 1);
    }
    return Angl_Err;
}

//临角处理32位带减速比计算
int32_t angle_limiting_int32(int32_t Angl_Error, int16_t buff, int16_t lap_encoder)
{
    if (Angl_Error < -(buff * (lap_encoder / 2)))
    {
        Angl_Error += (buff * (lap_encoder - 1));
    }
    if (Angl_Error > (buff * (lap_encoder / 2)))
    {
        Angl_Error -= (buff * (lap_encoder - 1));
    }
    return Angl_Error;
}

//过临界值复位码盘值 （限制于360度的码盘值循环 DJI电机）
int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder)
{
    if (value > (gear_Ratio * lap_encoder) / 2)
    {
        value = value - (gear_Ratio * lap_encoder);
    }
    if (value < (-(gear_Ratio * lap_encoder) / 2))
    {
        value = (gear_Ratio * lap_encoder) - value;
    }

    return value;
}

//电机堵转检测
static int16_t Block_Count = 0;

int16_t check_motor_block(int16_t position)
{
    static int16_t last_Position;
    if (int16_t_abs(last_Position - position) < 10)
        Block_Count++;
    else
        Block_Count = 0;
    last_Position = position;
    if (Block_Count > 100)
        return 1;
    else
        return 0;
}

//Y轴电机与底盘中心差角获取(以底盘中心为0度，向右为正角，向左为负角，大小范围为0~180度)
float Yaw_Different_Angle = 0;                    //云台底盘差角
int32_t Yaw_Middle_Code = (8192 * YAW_RATIO) / 2; //Y轴总码盘的半值

float get_yaw_different_angle(const motor_measure_t *yaw_position, int16_t Ratio)
{

    Yaw_Different_Angle = (yaw_position->actual_Position * 360) / (8192 * Ratio);

    return Yaw_Different_Angle;
}

/*
*多圈绝对值编码器数据转换
*传入值：真实码盘值
*/
int16_t rotate_50_cirecle = 0;
int16_t last_code = 0;

int16_t encoder_real(int32_t read_code)
{
    //	int code = 0;
    int code_output = 0;

    if (read_code <= 49152)
    {
        //		code = (read_code) / 3072;
        //		code_output = read_code - code * 3072;//将总圈数换算成单圈数

        code_output = read_code % 3072;
        return code_output;
    }
    else if (read_code > 49152)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/*
*功能：Yaw轴真实位置（多圈编码器）（左正右负）
*传入：读取到的真实单圈码盘值，单圈码盘总数，中间位置的码盘值
*传出：以中间位置为标准的临角处理位置值（左半圈正，右半圈负）
*/

int16_t yaw_actual_code_conversion(int16_t actual_code, int16_t max_code, int16_t middle_code)
{
    if (0 <= actual_code && actual_code <= middle_code) //将码盘值以中间为标准
    {
        last_code = max_code - (middle_code - actual_code);
        if (last_code >= (max_code / 2) && last_code <= max_code) //临角处理
            last_code = last_code - max_code;
        else if (last_code >= 0 && last_code < (max_code / 2))
            last_code = last_code;
    }
    else if (middle_code < actual_code && actual_code <= 3072)
    {
        last_code = actual_code - middle_code;
        if (last_code >= (max_code / 2) && last_code <= max_code) //临角处理
            last_code = last_code - max_code;
        else if (last_code >= 0 && last_code < (max_code / 2))
            last_code = last_code;
    }
    else
        last_code = -1;

    return last_code;
}

//void trigger_motor_turn_back(void)
//{
//    if (shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.speed_set = shoot_control.trigger_speed_set;
//    }
//    else
//    {
//        shoot_control.speed_set = -shoot_control.trigger_speed_set;
//    }

//    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.block_time++;
//        shoot_control.reverse_time = 0;
//    }
//    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
//    {
//        shoot_control.reverse_time++;
//    }
//    else
//    {
//        shoot_control.block_time = 0;
//    }
//}
