#ifndef __PID_H
#define __PID_H
#include "struct_typedef.h"


typedef enum
{
	PID_NORMAL = 0,  //����
	CHANGE_INTEGRAL, //�����
	STOP_SATURATED, //������
	
	PID_MANUAL, //�ֶ�ģʽ
} pid_mode_e;

typedef struct //pid�ṹ�����
{
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 SetValue;
    fp32 ActualValue;

    fp32 out;
	fp32 lastout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	
	fp32 Ierror;
    fp32 Derror[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3];   //����� 0���� 1��һ�� 2���ϴ�
	
	fp32 stepIn;
	
	/* ������� */
	fp32 deadband;
	
	/* ���ַ��� */
    fp32 epsilon; //���ַ�����(yu)ֵ
	
	/* ����ȫ΢�� */
	fp32 alpha;

    /* �����ֱ��� */
    fp32 maximum; //���ֵ
    fp32 minimum; //��Сֵ

    /* ����� */
    fp32 errorabsmax; //ƫ�����ֵ���ֵ
    fp32 errorabsmin; //ƫ�����ֵ��Сֵ

	/* ΢������ */
    fp32 gama; //΢�������˲�ϵ��
	
	pid_mode_e pid_mode;
} pid_parameter_t;


extern int32_t location_pid_int32(pid_parameter_t *pid , float actualValue);
extern void pid_init(pid_parameter_t *pid,fp32 kp,fp32 ki,fp32 kd,fp32 i_max,fp32 out_max);
extern void pid_clear(pid_parameter_t *pid);
extern float increment_pid(pid_parameter_t *pid , float actualValue);
/*����ʽPID�����趨ֵ����������*/
float step_in_processing(pid_parameter_t *vPID, float sp);
int32_t pid_regulator(pid_parameter_t *vPID , float actualValue);

#endif















