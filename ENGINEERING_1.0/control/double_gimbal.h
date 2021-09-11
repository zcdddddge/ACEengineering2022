#ifndef __DOUBLE_GIMBAL_H__
#define __DOUBLE_GIMBAL_H__
#include "struct_typedef.h"


typedef struct
{
	int16_t yaw_contral_data;  //y������� 
	int16_t pitch_contral_data;//p�������
	
	float   second_yaw_angle;  //����̨y����ʵ�Ƕ�
	float   second_pitch_angle;//����̨p����ʵ�Ƕ�
	float   last_second_yaw_angle; //�ϴ�y
	float   last_second_pitch_angle;//�ϴ�p
	
	int16_t yaw_middle_flag;   //y�ᾭ���м��־λ
	int16_t yaw_init_flag;     //y���ʼ����־λ
	int16_t pitch_block_flag;  //p���ת��־λ���������жϣ�
	int16_t pitch_block_count; //p���ת����
	int16_t pitch_init_flag;   //p���ʼ����־λ
	
}Second_Gimbal_Data_Typedef;

extern Second_Gimbal_Data_Typedef second_gimbal;

void Second_Gimbal_Control(int16_t yaw_control_data, int16_t pitch_contral_data);


#endif

