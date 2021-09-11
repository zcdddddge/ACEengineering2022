#ifndef __PARAMETER_H__
#define __PARAMETER_H__
#include "stm32f4xx.h" 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdint.h"


//��һ����ĸת��Ϊ��д
#define UPCASE( c ) ( ((c) >= 'a' && (c) <= 'z') ? ((c) - 0x20) : (c) )
//�ж��ַ��ǲ���10���Ƶ�����
#define DECCHK( c ) ((c) >= '0' && (c) <= '9')
//�õ�һ�������ĵ�ַ(word���)
#define B_PTR( var ) ( (byte *) (void *) &(var) )
#define W_PTR( var ) ( (word *) (void *) &(var) )
//��ֹ�����һ������
#define INC_SAT( val ) (val = ((val)+1 > (val)) ? (val)+1 : (val))
//��������Ԫ�صĸ���
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( (a[0]) ) )



/* ���ذ�����  || BOARD_GIMBAL / BOARD_CHASSIS */
#define BOARD_GIMBAL     //��������̨�廹�ǵ��̰�


//�������Ԥ����
#define		INFANTRY_3		3     // 3�Ų���
#define		INFANTRY_4		4     // 4�Ų���
#define		INFANTRY_5		5     // 5�Ų���
#define		INFANTRY_NUM	INFANTRY_3

/*****�������˵���̨��ֵ******/
#if (INFANTRY_NUM == INFANTRY_4)
#define Pitch_Middle_Angle  800  //   190   4�Ų���
#elif (INFANTRY_NUM == INFANTRY_3)
#define Pitch_Middle_Angle  145  //   190  3�Ų���
#endif



/*ģ�鹤������*/
#define WATCH_DOG                //�������Ź�
#define FIRE_WORK                //�䵯ģʽ���� (��������)
#define POWER_LIMIT              //������������


/*****�����Ƿ����������*****/
#define PITCH_GYRO_ANGULAR_SPEED  0//-(MPU6050_Real_Data.Gyro_Y) //P����ٶ�
#define YAW_GYRO_ANGULAR_SPEED    0//-(MPU6050_Real_Data.Gyro_Z) //Y����ٶ�       
#define YAW_POSTION_OUTPUT_FLAG   (1)
#define YAW_ANGLE_FLAG            (1)  //������λ�ö�Y��Ƕȵ�Ӱ��
#define YAW_SPEED_OUTPUT_FLAG     (-1)  //���ٶȻ�Y����ٶȷ���
#define PITCH_POSTION_OUTPUT_FLAG (-1)
#define PITCH_SPEED_OUTPUT_FLAG   (-1)


///*****������ͨ�������ѡ��PID����*********/
//#define test_short_focus  1      //�̽����
//#define test_long_focus   0      //�������
//#define test_industry     0      //��ҵ���


/************��� ������*���ٱ� ***************/
#define YAW_RATIO      (3*19)         //Yaw��
#define RESCUE_RATIO   (36)						//rescue
#define SPITCH_RATIO					 (36)					//small pitch
#define SYAW_RATIO						 (1)					//small yaw
#define FUNCTION_RATIO         (19)
#define LIFT_RATIO						(27)
//#define Sec_YAW_RATIO  (3*1)          //��Yaw��

#define RAD2DEG     (57.295779513082320876798154814105f)  // ����ת�Ƕ�


/* ���̵���ƶ��ٶ��趨 */
#define M3508_MAX_OUTPUT_CURRENT  (5000)   //m3508������������  
#define M2006_MAX_OUTPUT_CURRENT  (9500)   //m2006������������

#define MAX_MOTOR_CAN_INPUT    (2000.0f)   //3508����������
#define MAX_MOTOR_CAN_OUTPUT   (16000.0f)  //3508���������



/*��Ϣ���*/
#define NQ_DEBUG_ON 1

#define NQ_PRINTF(fmt, arg...) \
    do                         \
    {                          \
        if (NQ_DEBUG_ON)     \
            printf(fmt, ##arg); \
    } while (0)

	
	

#endif



