#ifndef __GYRO_ISR_H_
#define __GYRO_ISR_H_
#include "USART.h"

/*�ⲿ����*/
extern u8 USART6_hook;

typedef __packed struct
{
    float gx;
    float gy;
    float gz;
    float roll;
    float pitch;
	  float yaw;
}GYRO_t;



/*��ȡ���������ݽṹ���ַ*/
GYRO_t *Get_GYRO_Point(void);



/*����������У׼
	���ͳɹ�����0��ʧ�ܷ���1*/
u8 imu_calibration(void);



/*yaw�Ƕȹ���
	���ͳɹ�����0��ʧ�ܷ���1*/
u8 yaw_return_zero(void);


#endif
