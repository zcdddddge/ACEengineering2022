#include "pid.h"


#ifndef NULL
    #define NULL 0
#endif



/*超转函数*/
static fp32 OVERSHOOT(fp32 err, fp32 linit, fp32 reduce)
{
    if(err > linit)
    {
        err = err - reduce;
    }
    else if(err < -linit)
    {
        err = err + reduce;
    }

    return err;
}




/*************************************************************************************************
*名称:	PID_DEAL
*功能:	位置式PID算法
*形参: 	PID_t结构体 设定值Set	实际值Ref
*返回:	输出值
*说明:	无
*************************************************************************************************/
fp32 PID_DEAL(PID_t*PID, fp32 Set, fp32 Ref)
{
    if(PID == NULL)
    {
        return 0.0f;
    }

    PID->Set = Set;
    PID->Ref = Ref;

    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Set - PID->Ref;

    PID->TErr += PID->Err[0];

    PID->DErr[2] = PID->DErr[1];
    PID->DErr[1] = PID->DErr[0];
    PID->DErr[0] = PID->Err[0] - PID->Err[1];

    PID->Pout = PID->Kp * PID->Err[0];
    PID->Iout = PID->Ki * PID->TErr;
    PID->Dout = PID->Kd * PID->DErr[0];

    if(PID->Iout >= PID->MaxIout)
    {
        PID->Iout = PID->MaxIout;
    }

    if(PID->Iout <= -(PID->MaxIout))
    {
        PID->Iout = -(PID->MaxIout);
    }

    PID->Out = PID->Pout + PID->Iout + PID->Dout;

    if(PID->Out >= PID->MaxOut)
    {
        return PID->MaxOut;
    }

    if(PID->Out <= -(PID->MaxOut))
    {
        return (-PID->MaxOut);
    }

    return PID->Out;
}




/*************************************************************************************************
*名称:	PID_DEAL_OVERSHOOT
*功能:	位置式PID算法
*形参: 	PID_t结构体 设定值Set	实际值Ref
*返回:	输出值
*说明:	有超转处理
*************************************************************************************************/
fp32 PID_DEAL_OVERSHOOT(PID_t*PID, fp32 Set, fp32 Ref)
{
    if(PID == NULL)
    {
        return 0.0f;
    }

    PID->Set = Set;
    PID->Ref = Ref;

    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Set - PID->Ref;

    PID->Err[0] = OVERSHOOT(PID->Err[0], 360.0, 360.0);

    PID->TErr += PID->Err[0];

    PID->DErr[2] = PID->DErr[1];
    PID->DErr[1] = PID->DErr[0];
    PID->DErr[0] = PID->Err[0] - PID->Err[1];

    PID->Pout = PID->Kp * PID->Err[0];
    PID->Iout = PID->Ki * PID->TErr;
    PID->Dout = PID->Kd * PID->DErr[0];

    if(PID->Iout >= PID->MaxIout)
    {
        PID->Iout = PID->MaxIout;
    }

    if(PID->Iout <= -(PID->MaxIout))
    {
        PID->Iout = -(PID->MaxIout);
    }

    PID->Out = PID->Pout + PID->Iout + PID->Dout;

    if(PID->Out >= PID->MaxOut)
    {
        return PID->MaxOut;
    }

    if(PID->Out <= -(PID->MaxOut))
    {
        return (-PID->MaxOut);
    }

    return PID->Out;
}




/*************************************************************************************************
*名称:	PID_INCRE
*功能:	增量式PID算法
*形参: 	PID_t结构体 设定值Set	实际值Ref
*返回:	变化值
*说明:	未测试
*************************************************************************************************/
static fp32 PID_INCRE(PID_t*PID, fp32 Set, fp32 Ref)
{
    if(PID == NULL)
    {
        return 0.0f;
    }

    PID->Set = Set;
    PID->Ref = Ref;

    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Set - PID->Ref;

    PID->TErr += PID->Err[0];

    PID->DErr[2] = PID->DErr[1];
    PID->DErr[1] = PID->DErr[0];
    PID->DErr[0] = PID->Err[0] - PID->Err[1];

    PID->Pout = PID->Kp * ( PID->Err[0] - PID->Err[1] );
    PID->Iout = PID->Ki * PID->Err[0];
    PID->Dout = PID->Kd * ( PID->DErr[0] - 2 * PID->Err[1] + PID->Err[2]);

    if(PID->Iout >= PID->MaxIout)
    {
        PID->Iout = PID->MaxIout;
    }

    if(PID->Iout <= -(PID->MaxIout))
    {
        PID->Iout = -(PID->MaxIout);
    }

    PID->Out = PID->Pout + PID->Iout + PID->Dout;

    if(PID->Out >= PID->MaxOut)
    {
        return PID->MaxOut;
    }

    if(PID->Out <= -(PID->MaxOut))
    {
        return (-PID->MaxOut);
    }

    return PID->Out;
}

/*************************************************************************************************
*名称:	PID_ANTECE
*功能:	微分先行PID算法
*形参: 	PID_t结构体 设定值Set	实际值Ref
*返回:	输出值
*说明:	未测试，适用于输入值频繁变化，抑制输出振荡
*************************************************************************************************/
static fp32 PID_ANTECE(PID_t*PID, fp32 Set, fp32 Ref)
{
    static fp32 temp = 0.0f;

    if(PID == NULL)
    {
        return 0.0f;
    }

    PID->Set = Set;
    PID->Ref = Ref;

    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Set - PID->Ref;

    PID->TErr += PID->Err[0];

    temp = PID->DFilteGain * PID->Kd + PID->Kp;
    PID->DErr[2] = PID->Kd / temp;
    PID->DErr[1] = (PID->Kd + PID->Kp) / temp;
    PID->DErr[0] = PID->DFilteGain * PID->DErr[2];

    PID->Pout = PID->Kp * PID->Err[0];
    PID->Iout = PID->Ki * PID->TErr;
    PID->Dout = PID->DErr[0] * PID->Dout + PID->DErr[1] * PID->Ref + PID->DErr[2] * PID->LastRef;

    if(PID->Iout >= PID->MaxIout)
    {
        PID->Iout = PID->MaxIout;
    }

    if(PID->Iout <= -(PID->MaxIout))
    {
        PID->Iout = -(PID->MaxIout);
    }

    PID->Out = PID->Pout + PID->Iout + PID->Dout;

    if(PID->Out >= PID->MaxOut)
    {
        return PID->MaxOut;
    }

    if(PID->Out <= -(PID->MaxOut))
    {
        return (-PID->MaxOut);
    }

    PID->LastRef = Ref;

    return PID->Out;
}


/*************************************************************************************************
*名称:	PID_INCOMPL
*功能:	不完全微分PID算法
*形参: 	PID_t结构体 设定值Set	实际值Ref
*返回:	输出值
*说明:	未测试，据说比普通pid效果好
*************************************************************************************************/
fp32 PID_INCOMPL(PID_t*PID, fp32 Set, fp32 Ref)
{
    if(PID == NULL)
    {
        return 0.0f;
    }

    PID->Set = Set;
    PID->Ref = Ref;

    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Set - PID->Ref;

    PID->TErr += PID->Err[0];

    PID->DErr[2] = PID->DErr[1];
    PID->DErr[1] = PID->DErr[0];
    PID->DErr[0] = PID->Err[0] - PID->Err[1];

    PID->Pout = PID->Kp * PID->Err[0];
    PID->Iout = PID->Ki * PID->TErr;
    PID->Dout = (1 - PID->DFilteGain) * PID->Kd * PID->DErr[0] + PID->DFilteGain * PID->LastRef;

    if(PID->Iout >= PID->MaxIout)
    {
        PID->Iout = PID->MaxIout;
    }

    if(PID->Iout <= -(PID->MaxIout))
    {
        PID->Iout = -(PID->MaxIout);
    }

    PID->Out = PID->Pout + PID->Iout + PID->Dout;

    if(PID->Out >= PID->MaxOut)
    {
        return PID->MaxOut;
    }

    if(PID->Out <= -(PID->MaxOut))
    {
        return (-PID->MaxOut);
    }

    PID->LastRef = PID->Dout;

    return PID->Out;
}



/*************************************************************************************************
*名称:	PID_INIT
*功能:	PID结构体初始化
*形参: 	PID_t结构体 pid参数kp,ki,kd 输出限制max_Iout,max_out
*返回:	变化值
*说明:	未初始化 DFilteGain
*************************************************************************************************/
void PID_INIT(PID_t*PID, fp32 kp, fp32 ki, fp32 kd, fp32 max_Iout, fp32 max_out)
{
    if(PID == 0)
    {
        return;
    }

    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;

    PID->MaxIout = max_Iout;
    PID->MaxOut = max_out;

    PID->Err[2] = PID->Err[1] = PID->Err[0] = 0.0f;
    PID->DErr[2] = PID->DErr[1] = PID->DErr[0] = 0.0f;

    PID->Set = PID->Ref = PID->LastRef = PID->TErr = 0.0f;
    PID->Pout = PID->Iout = PID->Dout = PID->Out = 0.0f;

}
