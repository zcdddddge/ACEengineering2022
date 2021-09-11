#ifndef __PARAMETER_H__
#define __PARAMETER_H__
#include "stm32f4xx.h" 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdint.h"


//将一个字母转换为大写
#define UPCASE( c ) ( ((c) >= 'a' && (c) <= 'z') ? ((c) - 0x20) : (c) )
//判断字符是不是10进制的数字
#define DECCHK( c ) ((c) >= '0' && (c) <= '9')
//得到一个变量的地址(word宽度)
#define B_PTR( var ) ( (byte *) (void *) &(var) )
#define W_PTR( var ) ( (word *) (void *) &(var) )
//防止溢出的一个方法
#define INC_SAT( val ) (val = ((val)+1 > (val)) ? (val)+1 : (val))
//返回数组元素的个数
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( (a[0]) ) )



/* 主控板类型  || BOARD_GIMBAL / BOARD_CHASSIS */
#define BOARD_GIMBAL     //板子是云台板还是底盘板


//步兵编号预编译
#define		INFANTRY_3		3     // 3号步兵
#define		INFANTRY_4		4     // 4号步兵
#define		INFANTRY_5		5     // 5号步兵
#define		INFANTRY_NUM	INFANTRY_3

/*****各机器人的云台中值******/
#if (INFANTRY_NUM == INFANTRY_4)
#define Pitch_Middle_Angle  800  //   190   4号步兵
#elif (INFANTRY_NUM == INFANTRY_3)
#define Pitch_Middle_Angle  145  //   190  3号步兵
#endif



/*模块工作属性*/
#define WATCH_DOG                //启动看门狗
#define FIRE_WORK                //射弹模式开启 (开拨弹轮)
#define POWER_LIMIT              //启动功率限制


/*****陀螺仪方向参数配置*****/
#define PITCH_GYRO_ANGULAR_SPEED  0//-(MPU6050_Real_Data.Gyro_Y) //P轴角速度
#define YAW_GYRO_ANGULAR_SPEED    0//-(MPU6050_Real_Data.Gyro_Z) //Y轴角速度       
#define YAW_POSTION_OUTPUT_FLAG   (1)
#define YAW_ANGLE_FLAG            (1)  //陀螺仪位置对Y轴角度的影响
#define YAW_SPEED_OUTPUT_FLAG     (-1)  //纯速度环Y电机速度方向
#define PITCH_POSTION_OUTPUT_FLAG (-1)
#define PITCH_SPEED_OUTPUT_FLAG   (-1)


///*****测试普通自瞄相机选择PID参数*********/
//#define test_short_focus  1      //短焦相机
//#define test_long_focus   0      //长焦相机
//#define test_industry     0      //工业相机


/************电机 传动比*减速比 ***************/
#define YAW_RATIO      (3*19)         //Yaw轴
#define RESCUE_RATIO   (36)						//rescue
#define SPITCH_RATIO					 (36)					//small pitch
#define SYAW_RATIO						 (1)					//small yaw
#define FUNCTION_RATIO         (19)
#define LIFT_RATIO						(27)
//#define Sec_YAW_RATIO  (3*1)          //副Yaw轴

#define RAD2DEG     (57.295779513082320876798154814105f)  // 弧度转角度


/* 底盘电机移动速度设定 */
#define M3508_MAX_OUTPUT_CURRENT  (5000)   //m3508电机最大电流输出  
#define M2006_MAX_OUTPUT_CURRENT  (9500)   //m2006电机最大电流输出

#define MAX_MOTOR_CAN_INPUT    (2000.0f)   //3508最大电流输入
#define MAX_MOTOR_CAN_OUTPUT   (16000.0f)  //3508最大电流输出



/*信息输出*/
#define NQ_DEBUG_ON 1

#define NQ_PRINTF(fmt, arg...) \
    do                         \
    {                          \
        if (NQ_DEBUG_ON)     \
            printf(fmt, ##arg); \
    } while (0)

	
	

#endif



