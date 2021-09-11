#include "motor.h"

/*电机结构体清零*/
void MotorValZero(Motor_t *motor)
{
    motor->ID = motor->Pos_Lock = motor->Radio = 0;
    motor->ExpSpeed = 0;
    motor->ExpRadian = 0;
    PID_INIT(&motor->SPID, 0, 0, 0, 0, 0);
    PID_INIT(&motor->PPID, 0, 0, 0, 0, 0);
    EncoderValZero(motor->Encoder);
    motor->MotorType = CURRENCY_M;
}
