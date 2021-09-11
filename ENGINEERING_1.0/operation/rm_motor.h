#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H
#include "struct_typedef.h"
#include "pid.h"
#include "maths.h"
#include "math.h"
#include "CAN_1_Receive.h"

void motor_actual_position(motor_measure_t *rmMotor, int16_t gear_Ratio,int16_t lap_encoder); //计算真实码盘值
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder);                           //临角处理16位
int32_t angle_limiting_int32(int32_t Angl_Error,int16_t buff,int16_t lap_encoder);            //临角处理32位带减速比计算
int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder);               //过临界值复位码盘值
int16_t check_motor_block(int16_t position);                                                   //检测电机堵转
float get_yaw_different_angle(const motor_measure_t *yaw_position , int16_t Ratio);                  //获取云台与底盘差角
int16_t encoder_real(int32_t read_code);                                                       //多圈绝对值编码器数据转换
int16_t yaw_actual_code_conversion(int16_t actual_code , int16_t max_code , int16_t middle_code);//Yaw轴真实位置（多圈编码器）（左正右负）

int16_t motor_speed_control(pid_parameter_t *spid, int16_t setSpeed, int16_t actualSpeed, int16_t current_limit);  //电机速度闭环
int16_t motor_position_speed_control(pid_parameter_t *speed_pid, pid_parameter_t *position_pid, int16_t actual_position , int16_t actual_speed , int16_t setPosition, int16_t current_limit);
int16_t motor_position_Stepping(pid_parameter_t *speed_pid, pid_parameter_t *position_pid, int16_t actual_position, int16_t actual_speed, int16_t setPosition, int16_t current_limit);

#endif
