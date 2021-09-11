#ifndef __FILTER_H__
#define __FILTER_H__
#include "struct_typedef.h"
#include "visual.h"
#include "arm_math.h"



#define mat         arm_matrix_instance_f32 //�������ṹ�ľ���ṹ��   ( ���� �������������Լ�����(����) )
#define mat_init    arm_mat_init_f32        //��������ʼ��
#define mat_add     arm_mat_add_f32         //�������ӷ�
#define mat_sub     arm_mat_sub_f32         //����������
#define mat_mult    arm_mat_mult_f32        //�������˷�
#define mat_trans   arm_mat_trans_f32       //�������ת��
#define mat_inv     arm_mat_inverse_f32     //���������

typedef struct
{
    float raw_value;
    float filtered_value[4];
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, p, Pminus, k;
} kalman_filter_t;

typedef struct
{
    float raw_value;
    float filtered_value[4];
    float xhat_data[4], xhatminus_data[4], z_data[4], Pminus_data[16], K_data[16];
    float P_data[16];
    float AT_data[16], HT_data[16];
    float A_data[16];
    float H_data[16];
    float Q_data[16];
    float R_data[16];
} kalman_filter_init_t;




typedef struct
{
    float X_last; //��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
    float B;
    float Q;
    float R;
    float H;
} extKalman_t;



void KalmanCreate(extKalman_t *p, float T_Q, float T_R);
float KalmanFilter(extKalman_t* p, float dat);



void Kalman_Filter_Init(kalman_filter_t *F, kalman_filter_init_t *I);
void Kalman_Filter_Calc(kalman_filter_t *F, float signal1, float signal2);


extern kalman_filter_init_t kalman_Filter_Init;
extern kalman_filter_t kalman_Filter;



#endif
