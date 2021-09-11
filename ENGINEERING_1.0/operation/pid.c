#include "pid.h"     
#include "maths.h"
#include <stddef.h>
#include <math.h>

#pragma diag_suppress 177

static fp32 variable_integral_coefficient(fp32 error, fp32 absmax, fp32 absmin); //����ֺ���
static uint16_t beta_generation(float error, float epsilon); //���ַ��뺯��


int32_t pid_chassis_follow(pid_parameter_t *pid , float actualValue) 
{		
	//error
	pid->error[0] = pid->SetValue - actualValue;      

	if (fabs(pid->error[0]) > pid->deadband)
	{
		//������	
		//pid->Ierror += pid->error[0];
		pid->Ierror += (pid->error[0] + pid->error[1]) / 2;
		pid->Ierror *= variable_integral_coefficient(pid->error[0], pid->errorabsmax, pid->errorabsmin);
		//�����޷�
		pid->Ierror = int32_limit(pid->Ierror, 10000, -10000);
		
		//΢���� ��¼
		//pid->Derror[2] = pid->Derror[1];
		//pid->Derror[1] = pid->Derror[0];
		pid->Derror[0] = (pid->error[0] - pid->error[1]);
		
		//���pid����
		pid->out = (pid->Kp * pid->error[0]) + (pid->Ki * pid->Ierror) + (pid->Kd * pid->Derror[0]);  
	}
	else
	{
		pid_clear(pid);
	}
	
	//PIDÿһ�����
	pid->Pout = (pid->Kp * pid->error[0]);
	pid->Iout = (pid->Ki * pid->Ierror);
	pid->Dout = (pid->Kd * pid->Derror[0]);
	
	//��¼error
	pid->error[1] = pid->error[0];   
	
	return pid->out;
}


/*
*���ܣ��Դ������ݽ���λ��ʽpid���㲢���ֵ
*���룺1.PID�ṹ������   2.��ǰֵ
*�������Ը��豸�Ŀ�����
*˵����Ŀ��ֵ�ڱջ����ƺ����︳ֵ
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
	
	//������	
	//pid->Ierror += pid->error[0];
	pid->Ierror += (pid->error[0] + pid->error[1]) / 2;

	//�����޷�
	pid->Ierror = int32_limit(pid->Ierror, 5000, -5000);
	
	//΢���� ��¼
	//pid->Derror[2] = pid->Derror[1];
	//pid->Derror[1] = pid->Derror[0];
	pid->Derror[0] = (pid->error[0] - pid->error[1]);
	
	//���pid����
	pid->out = (pid->Kp * pid->error[0]) + (pid->Ki * pid->Ierror) + (pid->Kd * pid->Derror[0]);   
	
	//PIDÿһ�����
	pid->Pout = (pid->Kp * pid->error[0]);
	pid->Iout = (pid->Ki * pid->Ierror);
	pid->Dout = (pid->Kd * pid->Derror[0]);
	
	//��¼error
	pid->error[1] = pid->error[0];   
	
	return pid->out;
}



//����pid��ʼ��
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


//����ʽPID
float increment_pid(pid_parameter_t *pid , float actualValue)
{
	//error
	pid->error[0] = pid->SetValue - actualValue;
	
	//΢���� ��¼
	pid->Derror[0] = pid->error[0] - 2 * pid->error[1] + pid->error[2];
	
	//pid->Ierror = (pid->error[0] - pid->error[1]) / 2;
	pid->Ierror = pid->error[0];
	pid->Ierror *= variable_integral_coefficient(pid->error[0], pid->errorabsmax, pid->errorabsmin);
	
	//���pid����
	pid->out = (pid->Kp * (pid->error[0] - pid->error[1])) + (pid->Ki * pid->Ierror) + (pid->Kd * pid->Derror[0]);   
	
	//PIDÿһ�����
	pid->Pout = (pid->Kp * (pid->error[0] - pid->error[1]));
	pid->Iout = (pid->Ki * pid->Ierror);
	pid->Dout = (pid->Kd * pid->Derror[0]);
	
	//��¼error
	pid->error[2] = pid->error[1]; //��¼���ϴ����
	pid->error[1] = pid->error[0]; //��¼�ϴ����
	
	return pid->out;
}




///**
//  * @brief          �����ֱ���PID������
//  * @param[in]      pid: pid�ṹ��
//  * @param[in]      actualValue: ʵ��ֵ
//  * @retval         �Ը��豸�Ŀ�����
//  * @attention      Ҫ�õ� maximum �� minimum 
//  */
//int32_t pid_antiintegral_saturation(pid_parameter_t *vPID, fp32 actualValue)
//{
//    int32_t dError;

//    //error
//    vPID->error[0] = vPID->SetValue - actualValue;

//    //������
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
//    //�����޷�
//    vPID->Ierror = int32_limit(vPID->Ierror, vPID->max_iout, -vPID->max_iout);

//    //΢��
//    dError = vPID->error[0] - vPID->error[1];

//    //΢���� ��¼
//    vPID->Derror[2] = vPID->Derror[1];
//    vPID->Derror[1] = vPID->Derror[0];
//    vPID->Derror[0] = (vPID->error[0] - vPID->error[1]);

//    vPID->out = (vPID->Kp * vPID->error[0]) + (vPID->Ki * vPID->Ierror) + (vPID->Kd * dError);

//    //PIDÿһ�����
//    vPID->Pout = (vPID->Kp * vPID->error[0]);
//    vPID->Iout = (vPID->Ki * vPID->Ierror);
//    vPID->Dout = (vPID->Kd * dError);

//    //��¼error
//    vPID->error[1] = vPID->error[0];

//    return vPID->out;
//}

/**
  * @brief          �����ϵ����������ʵ��һ�����0��1֮��ķֶ����Ժ���
  * @param[in]      error: ��ǰ�����ƫ��ֵ
  * @param[in]      absmax: ƫ�����ֵ�����ֵ
  * @param[in]      absmin: ƫ�����ֵ����Сֵ
  * @retval         factor: ���ƻ��ֵı���
  * @attention      ��ƫ��ľ���ֵС����Сֵʱ�����Ϊ1����ƫ��ľ���ֵ�������ֵʱ�����Ϊ0
  *                 ��ƫ��ľ���ֵ�������ֵ����Сֵ֮��ʱ�������0��1֮�����б仯
  */
static fp32 variable_integral_coefficient(fp32 error, fp32 absmax, fp32 absmin)
{
    fp32 factor = 0.0f;

    if (float_abs(error) <= absmin) //��Сֵ
    {
        factor = 1.0f;
    }
    else if (float_abs(error) > absmax) //���ֵ
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
  * @brief          ���ַ��뺯��
  * @param[in]      error: ��ǰ�����ֵ
  * @param[in]      epsilon: �����ֵ
  * @retval         1: ���ϻ�����
  *                 0: ȥ��������
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
//  * @brief          ΢������PID
//  * @param[in]      pid: pid�ṹ��
//  * @param[in]      actualValue: ʵ��ֵ
//  * @retval         �Ը��豸�Ŀ�����
//  * @attention      ��ĳЩ����ֵƵ���Ҵ���仯�ĳ��ϣ�΢�����������ϵͳ���񵴡�Ϊ����Ӧ���ָ���ֵƵ���仯�ĳ��ϣ����������΢�������㷨��
//  *                 Ҫ�õ� gama 
//  */
//int32_t PIDRegulation(pid_parameter_t *vPID, fp32 actualValue)
//{
//    fp32 c1, c2, c3, temp;

//    //error
//    vPID->error[0] = vPID->SetValue - actualValue;

//    /******************������******************/
//    /* �������� */
//    //vPID->Ierror += vPID->error[0];  //��������
//    /* ���λ��� */
//    vPID->Ierror += (vPID->error[0] + vPID->error[1]) / 2;
//    /* �����PID */
//    //fp32 x = variable_integral_coefficient(vPID->error[0], vPID->errorabsmax, vPID->errorabsmin);
//    //vPID->Ierror += x * (vPID->error[0] + vPID->error[1]) / 2; //���λ���

//    //�����޷�
//    vPID->Ierror = int32_limit(vPID->Ierror, vPID->max_iout, -vPID->max_iout);

//    //΢���� ��¼
//    temp = vPID->gama * vPID->Kd + vPID->Kp;
//    c3 = vPID->Kd / temp;
//    c2 = (vPID->Kd + vPID->Kp) / temp;
//    c1 = vPID->gama * c3;
//    vPID->Derror[0] = c1 * vPID->Derror[0] + c2 * actualValue + c3 * vPID->Last_ActualValue;

//    //���pid����
//    vPID->out = (vPID->Kp * vPID->error[0]) + (vPID->Ki * vPID->Ierror) + (vPID->Kd * vPID->Derror[0]);

//    //PIDÿһ�����
//    vPID->Pout = (vPID->Kp * vPID->error[0]);
//    vPID->Iout = (vPID->Ki * vPID->Ierror);
//    vPID->Dout = (vPID->Kd * vPID->Derror[0]);

//    //��¼error
//    vPID->error[1] = vPID->error[0];
//    //��¼ʵ��ֵ
//    vPID->Last_ActualValue = actualValue;

//    return vPID->out;
//}

///**
//  * @brief          ʵ��ǰ��������
//  * @param[in]      vFFC: FFC�ṹ��
//  * @retval         �Ը��豸�Ŀ�����
//  * @attention      �������ǰ����������ֻ��Ҫ��PID������������ϲ���һ��ͺ���U(k)=Up(k)+Uf(k)
//  *                 ʹ��ǰ: vFFC->rin=vPID-> setpoint
//  *                 Ҫ�õ� Alpha �� Beta
//  */
//fp32 FeedforwardController(FFC *vFFC)
//{
//    fp32 result;

//    result = vFFC->Alpha * (vFFC->rin - vFFC->lastRin) + vFFC->Beta * (vFFC->rin - 2 * vFFC->lastRin + vFFC->perrRin);

//    vFFC->perrRin = vFFC->lastRin;

//    vFFC->lastRin = vFFC->rin;

//    return result;
//}



/*����ʽPID�����趨ֵ����������*/
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


// NO_4 �޸�����
int32_t pid_regulator(pid_parameter_t *vPID , float actualValue)
{
	//error
	vPID->error[0] = vPID->SetValue - actualValue;   

//	if (fabs(vPID->error[0]) <= vPID->deadband)
//	{
//		vPID->error[0] = 0.0f;
//	}	
	
	//������	
//	if (beta_generation(vPID->error[0], 5.0f) == 1)
//	{
//		
//	}
	//vPID->Ierror += (vPID->error[0] + vPID->error[1]) / 2; //���λ���
	
//	//maximum��I  
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
	
	//����� TODO
	vPID->Ierror += (vPID->error[0] + vPID->error[1]) / 2;
	vPID->Ierror *= variable_integral_coefficient(vPID->error[0], vPID->errorabsmax, vPID->errorabsmin);
	
	//�����޷�
	vPID->Ierror = int32_limit(vPID->Ierror, vPID->max_iout, -vPID->max_iout);
	
	//΢���� ��¼
	//vPID->Derror[2] = vPID->Derror[1];
	//vPID->Derror[1] = vPID->Derror[0];
	vPID->Derror[0] = (vPID->error[0] - vPID->error[1]);
	
	//PIDÿһ�����
	vPID->Pout = (vPID->Kp * vPID->error[0]);
	vPID->Iout = (vPID->Ki * vPID->Ierror);
	vPID->Dout = (vPID->Kd * vPID->Derror[0]);
	//����΢��������������ȫ΢��
	//vPID->Dout = (vPID->Kd * vPID->Derror[0] * (1 - vPID->alpha)) + (vPID->alpha * vPID->Dout);
	
//	if (fabs(vPID->error[0]) <= vPID->deadband)
//	{
//		vPID->error[0] = vPID->error[1] = vPID->error[2] = 0.0f;
//		vPID->Derror[0] = vPID->Derror[1] = vPID->Derror[2] = 0.0f;
//		vPID->out = vPID->Pout = vPID->Iout = vPID->Dout = 0.0f;
//	}
	
	//���PID����
	vPID->out = vPID->Pout + vPID->Iout + vPID->Dout;   
	
	//��¼error
	vPID->error[1] = vPID->error[0];
	
	return vPID->out;
}








