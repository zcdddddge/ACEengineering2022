#ifndef __GIMBALMOTOR_H_
#define __GIMBALMOTOR_H_
#include "motor.h"
#include "RemoteDeal.h"


/*云台结构体*/
typedef __packed struct
{
	Motor_t PitchMotor;
	Motor_t YawMotor;
	REMOTE_t *Rc ;
	void (*Can_Send_Gimbal)(int16_t,int16_t);
	void (*Can_Send_Y)(int16_t );
}G_t;





/*PY轴电机初始化*/
void PY_Motor_Init(G_t *G);


/*PY轴码盘控制模式*/
void PY_Encoder_DRIVE(G_t *G,float P,float Y,float Y_SPEED);

void PY_Motor(G_t *G );

/*云台断电*/
void Gimbal_Poweroff(G_t *G);


/*拨弹电机初始化*/
void Ammuniti_Motor_Init(G_t *G);


/*拨弹电机控制*/
void Ammuniti_Ctrl(G_t *G,int8_t SpeedGain);

#endif
