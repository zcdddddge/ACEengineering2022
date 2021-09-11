/*
 *
 * 　　┏┓　　　┏┓+ +
 * 　┏┛┻━━━┛┻┓ + +
 * 　┃　　　　　　　┃ 　
 * 　┃　　　━　　　┃ ++ + + +
 *  ████━████ ┃+
 * 　┃　　　　　　　┃ +
 * 　┃　　　┻　　　┃
 * 　┃　　　　　　　┃ + +
 * 　┗━┓　　　┏━┛
 * 　　　┃　　　┃　　　　　　　　　　　
 * 　　　┃　　　┃ + + + +
 * 　　　┃　　　┃
 * 　　　┃　　　┃ +  神兽保佑
 * 　　　┃　　　┃    代码无bug　　
 * 　　　┃　　　┃　　+　　　　　　　　　
 * 　　　┃　 　　┗━━━┓ + +
 * 　　　┃ 　　　　　　　┣┓
 * 　　　┃ 　　　　　　　┏┛
 * 　　　┗┓┓┏━┳┓┏┛ + + + +
 * 　　　　┃┫┫　┃┫┫
 * 　　　　┗┻┛　┗┻┛+ + + +
 *
 */


#include "motor.h"
/**
 * @description: 电机结构体清零
 * @param {Motor_t} *motor
 * @return {*}
 */
void MotorValZero(Motor_t *motor)
{
    motor->ID = motor->Pos_Lock = motor->Radio = 0;
    motor->ExpSpeed 	= 0;
    motor->ExpRadian 	= 0;
    motor->state			= 0;
    PID_INIT(&motor->SPID, 0, 0, 0, 0, 0);
    PID_INIT(&motor->PPID, 0, 0, 0, 0, 0);
    EncoderValZero(motor->Encoder);
    motor->MotorType = CURRENCY_M;
    motor->ResetFlag = 0;
    //motor->clock = motor->lock = motor->dire=0 ;
}
