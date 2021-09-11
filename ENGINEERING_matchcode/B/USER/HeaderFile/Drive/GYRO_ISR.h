#ifndef __GYRO_ISR_H_
#define __GYRO_ISR_H_
#include "USART.h"

/*外部变量*/
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



/*获取陀螺仪数据结构体地址*/
GYRO_t *Get_GYRO_Point(void);



/*陀螺仪重新校准
	发送成功返回0，失败返回1*/
u8 imu_calibration(void);



/*yaw角度归零
	发送成功返回0，失败返回1*/
u8 yaw_return_zero(void);


#endif
