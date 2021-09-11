#ifndef __PID_H_
#define __PID_H_
#include "stm32f4xx.h"

#define fp32 float
	
/*PID结构体*/
typedef __packed struct
{	
	fp32 Set;
	fp32 Ref;
	fp32 LastRef;
	
	fp32 Kp;
	fp32 Ki;
	fp32 Kd;
	
	fp32 Err[3];
	fp32 TErr;
	fp32 DErr[3];
	
	fp32 Pout;
	fp32 Iout;
	fp32 Dout;
	fp32 Out;
	
	fp32 MaxOut;
	fp32 MaxIout;
	
	fp32 DFilteGain;
}PID_t;


/*带超转处理的PID函数，用于电机位置环时超过360度的情况*/
fp32 PID_DEAL(PID_t*PID,fp32 Set,fp32 Ref);


/*不带超转处理的PID函数，用于速度环和电机位置环时不超过360度的情况*/
fp32 PID_DEAL_OVERSHOOT(PID_t*PID,fp32 Set,fp32 Ref);


/*PID结构体初始化函数*/
void PID_INIT(PID_t*PID,fp32 kp,fp32 ki,fp32 kd,fp32 max_Iout,fp32 max_out);

#endif
