#ifndef __VISUAL_H__
#define __VISUAL_H__
#include "struct_typedef.h"



/*****视觉相关结构体****/
typedef struct
{
    float auto_pitch_angle;      //视觉传回P轴差角
    float auto_yaw_angle;        //视觉传回Y轴差角
	
	float auto_kalman_pitch_angle;      //视觉传回P轴差角
    float auto_kalman_yaw_angle;        //视觉传回Y轴差角
	
    float last_Auto_Pitch_Angle; //上一次的P轴差角
    float last_Auto_Yaw_Angle;   //上一次的Y轴差角
    float len;                   //视觉传回距离
	int8_t fire;                 //是否开火
	
    float auto_yaw_speed;        //视觉回传数据算出的角速度
    float auto_pitch_sum;        //传回P轴角度积分
    float auto_pitch_angle_kf;   //电控卡尔曼处理后的P轴差角
    float auto_yaw_angle_kf;     //电控卡尔曼处理后的Y轴差角

    int16_t auto_lost_data_count;  //丢失目标计数
    int16_t auto_lost_data_flag;   //丢失目标标志位
	int16_t auto_yaw_zero_flag;    //yaw轴0值标志位

    float pitch_control_data;     //P轴视觉控制量
    float yaw_control_data;       //y轴视觉控制量

} Vision_Auto_Data_t;


//返回视觉自瞄控制变量，通过指针传递方式传递信息
extern Vision_Auto_Data_t *Get_Auto_Control_Point(void);
extern void automatic_aiming_init(void);

extern void MiniPC_Kalman_Data_Init(void);
extern void visual_send_data(u8 data, u8 mode, u8 shoot_speed);
extern void visual_data_reception(void);
extern float hex2Float(uint8_t HighByte, uint8_t LowByte);

#endif
