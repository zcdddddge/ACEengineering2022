#ifndef __MATHS_H
#define __MATHS_H
#include "struct_typedef.h"



//�˶����ٶ�����б�º���
typedef __packed struct
{
    float Input;      //��ǰȡ��ֵ
    float Last_Input; //�ϴ�ȡ��ֵ
    float Output;     //���ֵ
    float acc_now;    //��ǰ���ٶ�
    float acc_limit;  //��Ҫ���Ƶļ��ٶ�
} acceleration_control_type_t;

//������ֵ�˲����������㣩
typedef __packed struct
{
    fp32 Input;     //��ǰȡ��ֵ
    int32_t count_num;   //ȡ������
    fp32 Output;    //�˲����
    fp32 Sum;       //�ۼ��ܺ�
    fp32 FIFO[250]; //����
    int32_t sum_flag;    //�Ѿ���250����־
} sliding_mean_filter_type_t;

//һ�׵�ͨ�˲�����
typedef __packed struct
{
    fp32 input;        //��������
    fp32 last_input;   //�ϴ�����
    fp32 out;          //�˲����������
    fp32 num;          //�˲�����
} first_order_filter_type_t;



int32_t int32_limit(int32_t x, int32_t max, int32_t min);
int16_t int16_limit(int16_t x, int16_t max, int16_t min);
fp32 float_limit(fp32 x, fp32 max, fp32 min);
signed long limit_long(signed long x, signed long max, signed long min);
int16_t int16_t_abs(int16_t x);
signed long long_abs(signed long x);
fp32 float_abs(fp32 x);			// ����ֵ����
int16_t max_abs(int16_t x, int16_t y); //ȡ���ֵ�ľ���ֵ
fp32 invSqrt(fp32 x);       //ƽ��������

/* �˶�����б�º��������ٶ����ƣ���16λ�� */
int16_t motion_acceleration_control(acceleration_control_type_t *acceleration_control, int16_t Input, int16_t Limit); //�˶����ٶ�����

/* ��ͨ�˲� */
extern float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);

/* ƽ���˲� */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, fp32 Input, int32_t num);  //��ֵ�����˲�
void sliding_mean_filter_init(sliding_mean_filter_type_t *mean_filter);  //��ֵ�����˲���ʼ�����ɲ��ã�ֱ�Ӷ���ṹ��ʱ����ֵ��

/* ѭ������ */
int16_t loop_restriction_int16(int16_t num, int16_t limit_num);          //16λѭ���޷�
float loop_restriction_float(float num, float limit_num);                //����ѭ���޷�
float loop_fp32_constrain(float Input, float minValue, float maxValue);  //ѭ�����ƣ���̨�Ƕȴ���
float cos_calculate(float angle);
float sin_calculate(float angle);


/* б�º��� */
void data_accelerated_control(float *input, float acc);  //���ٶ�����б�º���

//���ȸ�ʽ��Ϊ-PI~PI
#define RAD_FORMAT(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif

