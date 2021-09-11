#ifndef __IMU_H__
#define __IMU_H__
#include "struct_typedef.h"
#include "stm32f4xx.h"
/* 
SendBuff[0] = 0xfe;
SendBuff[1] = send_1 >> 8;
SendBuff[2] = send_1;
SendBuff[3] = send_2 >> 8;
SendBuff[4] = send_2;
SendBuff[5] = send_3 >> 8;
SendBuff[6] = send_3;
SendBuff[7] = send_4 >> 8;
SendBuff[8] = send_4;
SendBuff[9] = send_5 >> 8;
SendBuff[10] = send_5;
SendBuff[11] = send_6 >> 8;
SendBuff[12] = send_6;
SendBuff[13] = 0xee;
*/


typedef struct
{
	uint16_t length;
	
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
	
    float Temp;     //转换成实际的温度，单位为摄氏度
	
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
	
    float yaw_angle;
    float pitch_angle;
    float roll_angle;
}imu_data_t;


typedef __packed struct
{
	u8 data[128];
	float Roll;
	float Pitch;
	float	Yaw;
	float Yaw_Lock;
	float Yaw_Var;
	u8 flag;
}PYR_t;

/*返回底盘PYR数据指针*/
PYR_t* Return_PYR_t(void);




const imu_data_t *get_imu_control_point(void);
void imu_control_init(void);

void imu_data_deal(void);
void imu_zero_correct(void);
void gyro_usart_iwdg(void);
void imu_data_reception(void);

#endif
