#ifndef __PID_H_
#define __PID_H_
#include "stm32f4xx.h"

#define fp32 float
	
/*PID�ṹ��*/
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


/*����ת�����PID���������ڵ��λ�û�ʱ����360�ȵ����*/
fp32 PID_DEAL(PID_t*PID,fp32 Set,fp32 Ref);


/*������ת�����PID�����������ٶȻ��͵��λ�û�ʱ������360�ȵ����*/
fp32 PID_DEAL_OVERSHOOT(PID_t*PID,fp32 Set,fp32 Ref);


/*PID�ṹ���ʼ������*/
void PID_INIT(PID_t*PID,fp32 kp,fp32 ki,fp32 kd,fp32 max_Iout,fp32 max_out);

#endif
