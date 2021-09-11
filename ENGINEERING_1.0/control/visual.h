#ifndef __VISUAL_H__
#define __VISUAL_H__
#include "struct_typedef.h"



/*****�Ӿ���ؽṹ��****/
typedef struct
{
    float auto_pitch_angle;      //�Ӿ�����P����
    float auto_yaw_angle;        //�Ӿ�����Y����
	
	float auto_kalman_pitch_angle;      //�Ӿ�����P����
    float auto_kalman_yaw_angle;        //�Ӿ�����Y����
	
    float last_Auto_Pitch_Angle; //��һ�ε�P����
    float last_Auto_Yaw_Angle;   //��һ�ε�Y����
    float len;                   //�Ӿ����ؾ���
	int8_t fire;                 //�Ƿ񿪻�
	
    float auto_yaw_speed;        //�Ӿ��ش���������Ľ��ٶ�
    float auto_pitch_sum;        //����P��ǶȻ���
    float auto_pitch_angle_kf;   //��ؿ�����������P����
    float auto_yaw_angle_kf;     //��ؿ�����������Y����

    int16_t auto_lost_data_count;  //��ʧĿ�����
    int16_t auto_lost_data_flag;   //��ʧĿ���־λ
	int16_t auto_yaw_zero_flag;    //yaw��0ֵ��־λ

    float pitch_control_data;     //P���Ӿ�������
    float yaw_control_data;       //y���Ӿ�������

} Vision_Auto_Data_t;


//�����Ӿ�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
extern Vision_Auto_Data_t *Get_Auto_Control_Point(void);
extern void automatic_aiming_init(void);

extern void MiniPC_Kalman_Data_Init(void);
extern void visual_send_data(u8 data, u8 mode, u8 shoot_speed);
extern void visual_data_reception(void);
extern float hex2Float(uint8_t HighByte, uint8_t LowByte);

#endif
