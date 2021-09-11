/**
  ******************************************************************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务。
  ******************************************************************************
  */

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__
#include "pid.h"
#include "rc.h"
#include "can_1_receive.h"
#include "visual.h"
#include "vl53l0.h"
#include "gpio_deal.h"
#include "shoot_task.h"
#include "maths.h"
#include "imu.h"
#include "parameter.h"

/*OS控制任务周期以及启动时间*/
#define GIMBAL_TASK_INIT_TIME 	1
#define GIMBAL_CONTROL_TIME_MS  2

/*抬升*/
#define UPLIFT_S_P   	4.0f
#define UPLIFT_S_I   	0.0f
#define UPLIFT_S_D   	0.2f
#define UPLIFT_P_P  	400.0f  // 350.5 
#define	UPLIFT_P_I   	0.0f
#define UPLIFT_P_D   	100.0f

/*平移*/
#define TRANSLATION_S_P 1.22f
#define TRANSLATION_S_I 0.0f
#define TRANSLATION_S_D 0.2f
#define TRANSLATION_P_P 100.0f
#define TRANSLATION_P_I 0.0f
#define TRANSLATION_P_D 0.0f

/*伸缩*/
#define TELESCOP_S_P   	1.22f
#define TELESCOP_S_I   	0.0f
#define TELESCOP_S_D   	0.02f
#define TELESCOP_P_P  		100.5f
#define	TELESCOP_P_I   	0.0f
#define TELESCOP_P_D   	0.0f

/*夹紧*/
#define CLIP_S_P   	30.5f
#define CLIP_S_I   	0.0f
#define CLIP_S_D   	0.02f
#define CLIP_P_P  	20.7f
#define	CLIP_P_I   	0.0f
#define CLIP_P_D   	0.0f

/*翻转*/
#define FLIP_S_P   	4.0f
#define FLIP_S_I   	0.0f
#define FLIP_S_D   	0.05f
#define FLIP_P_P  	100.0f
#define	FLIP_P_I   	0.0f
#define FLIP_P_D   	50.0f

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE    1

#if (INFANTRY_NUM == INFANTRY_4)
/**********************Pitch轴PID参数**********************/
#if (PITCH_PID_MODE == 1)
	#define GIMBAL_UP_P_PITCH_P  41.5f   //P位置环  53   410    160       58
	#define GIMBAL_UP_P_PITCH_I  0.0f     //8f      
	#define GIMBAL_UP_P_PITCH_D  60.0f     //0.0f       360       100     100

	#define GIMBAL_UP_S_PITCH_P  8.5f   //P速度环(不要加i)       1.5     9.5
	#define GIMBAL_UP_S_PITCH_I  0.0f      
	#define GIMBAL_UP_S_PITCH_D  3.0f   

	#define GIMBAL_DOWN_P_PITCH_P  28.0f   //P位置环  53   410    160       58
	#define GIMBAL_DOWN_P_PITCH_I  0.0f     //8f      
	#define GIMBAL_DOWN_P_PITCH_D  200.0f     //0.0f       360       100     100

	#define GIMBAL_DOWN_S_PITCH_P  7.5f   //P速度环(不要加i)       1.5     9.5
	#define GIMBAL_DOWN_S_PITCH_I  0.0f   
	#define GIMBAL_DOWN_S_PITCH_D  10.8f   
#elif (PITCH_PID_MODE == 2)
	#define GIMBAL_UP_P_PITCH_P  50.5f   //P位置环  53   410    160       58
	#define GIMBAL_UP_P_PITCH_I  0.0f     //8f      
	#define GIMBAL_UP_P_PITCH_D  60.0f     //0.0f       360       100     100

	#define GIMBAL_UP_S_PITCH_P  10.5f   //P速度环(不要加i)       1.5     9.5
	#define GIMBAL_UP_S_PITCH_I  0.0f      
	#define GIMBAL_UP_S_PITCH_D  3.0f   

	#define GIMBAL_DOWN_P_PITCH_P  30.0f   //P位置环  53   410    160       58
	#define GIMBAL_DOWN_P_PITCH_I  0.0f     //8f      
	#define GIMBAL_DOWN_P_PITCH_D  50.0f     //0.0f       360       100     100

	#define GIMBAL_DOWN_S_PITCH_P  7.5f   //P速度环(不要加i)       1.5     9.5
	#define GIMBAL_DOWN_S_PITCH_I  0.0f   
	#define GIMBAL_DOWN_S_PITCH_D  20.8f   
#elif (PITCH_PID_MODE == 3)
	#define GIMBAL_P_PITCH_P  60.0f   //P位置环  70   
	#define GIMBAL_P_PITCH_I  1.5f     // 1.1f
	#define GIMBAL_P_PITCH_D  75.0f     //0.0f   75

	#define GIMBAL_S_PITCH_P  10.5f   //P速度环(不要加i)       1.5     9.5
	#define GIMBAL_S_PITCH_I  0.0f      
	#define GIMBAL_S_PITCH_D  6.0f    //3.5  4
#endif

/**********************Yaw轴PID参数**********************/
#define GIMBAL_P_YAW_P  160.0f   //Y位置环    150     62    130
#define GIMBAL_P_YAW_I  0.0f       
#define GIMBAL_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_S_YAW_P  12.0f     //Y速度环     8      10     9
#define GIMBAL_S_YAW_I  0.0f   
#define GIMBAL_S_YAW_D  0.2f     //2                   0

#define GIMBAL_ROTATE_P_YAW_P  150.0f   //Y位置环    150     62    130
#define GIMBAL_ROTATE_P_YAW_I  0.0f       
#define GIMBAL_ROTATE_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_ROTATE_S_YAW_P  9.0f     //Y速度环     8      10     9
#define GIMBAL_ROTATE_S_YAW_I  0.0f   
#define GIMBAL_ROTATE_S_YAW_D  0.0f     //2                   0

/**********************低通滤波比例**********************/
#define Gimbal_Pitch_Fir_Ord_Low_Fil_Param  0.0884f

/**********************云台pitch和yaw角度限制**********************/
#define PITCH_ANGLE_LIMIT_UP    38
#define PITCH_ANGLE_LIMIT_DOWN  (-22)
#define YAW_ANGLE_LIMIT         130

/**********************运动加速度限制**********************/
#define GIMBAL_PITCH_ACCELERAD    2         //云台俯仰加速度限制
#define GIMBAL_YAW_ACCELERAD      2         //云台偏航加速度限制
#define GIMBAL_AUTO_YAW_ACCELERAD 1         //云台自动偏航加速度限制

/**********************pitch和yaw输出量限制**********************/
#define YAW_OUTPUT_LIMIT         11000 
#define YAW_INIT_OUTPUT_LIMIT    8000
#define PITCH_OUTPUT_LIMIT       8000 
#define PITCH_INIT_OUTPUT_LIMIT  5000

/**********************键盘鼠标遥控速度设置**********************/
#define MOUSE_YAW_SPEED       0.011f    //鼠标yaw轴速度增益     0.021f 
#define MOUSE_PITCH_SPEED     0.009f    //鼠标pitch轴速度增益  0.13
#define RC_YAW_SPEED          0.0026f   //遥控器yaw轴速度增益
#define RC_PITCH_SPEED        0.0026f   //遥控器pitch轴速度增益 0.0026



/*********视觉PY轴数据pid参数定义***************/

/**后面视觉需要三套pid，自瞄短焦，自瞄工业，打符工业**/

//*************短焦摄像头4mm
////P轴  x1.7f
//#define GIMBAL_AUTO_SHORT_P_PITCH_P 60.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_SHORT_P_PITCH_I 0.5f      
//#define GIMBAL_AUTO_SHORT_P_PITCH_D 70.2f      

//#define GIMBAL_AUTO_SHORT_S_PITCH_P 8.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_SHORT_S_PITCH_D 2.2f      //2
//P轴
#define GIMBAL_AUTO_SHORT_P_PITCH_P 15.5f     //P自动位置环   14
#define GIMBAL_AUTO_SHORT_P_PITCH_I 1.0f      
#define GIMBAL_AUTO_SHORT_P_PITCH_D 15.0f      // 40*40=1600   40.2f   15.0f

#define GIMBAL_AUTO_SHORT_S_PITCH_P 7.5f     //P自动速度环   13.5
#define GIMBAL_AUTO_SHORT_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_SHORT_S_PITCH_D 45.0f      //2   45.0f  
////上升
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_P 63.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_D 10.1f      

//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_P 10.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_D 0.0f  
////下降
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_P 22.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_D 9.1f      

//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_P 7.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_D 0.0f  
////上升
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_P 60.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_D 10.0f      

//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_P 8.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_D 0.0f  
////下降
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_P 21.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_D 20.1f      

//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_P 8.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_D 0.0f  
//Y轴
#define GIMBAL_AUTO_SHORT_P_YAW_P 170.0f       //Y自动位置环   160
#define GIMBAL_AUTO_SHORT_P_YAW_I 3.0f        // 2
#define GIMBAL_AUTO_SHORT_P_YAW_D 12.0f        //16

#define GIMBAL_AUTO_SHORT_S_YAW_P 24.6f       //Y自动速度环    7.6
#define GIMBAL_AUTO_SHORT_S_YAW_I 0.0f        
#define GIMBAL_AUTO_SHORT_S_YAW_D 0.0f        

//*************工业摄像头
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_P 30.0f    //P自动位置环  530.0f   35
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

#define GIMBAL_AUTO_INDUSTRY_S_PITCH_P 8.0f     //P自动速度环  12
#define GIMBAL_AUTO_INDUSTRY_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_INDUSTRY_S_PITCH_D 0.0f      //5
//上升
#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_P 60.0f    //P自动位置环  530.0f   35
#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_P 10.0f     //P自动速度环  12
#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_D 0.0f      //5
//下降
#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_P 26.0f    //P自动位置环  530.0f   35
#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_P 8.0f     //P自动速度环  12
#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_D 0.0f      //5
//yaw
#define GIMBAL_AUTO_INDUSTRY_P_YAW_P 150.0f       //Y自动位置环 170   130   100
#define GIMBAL_AUTO_INDUSTRY_P_YAW_I 0.0f        
#define GIMBAL_AUTO_INDUSTRY_P_YAW_D 15.0f        //15

#define GIMBAL_AUTO_INDUSTRY_S_YAW_P 20.0f       //Y自动速度环 12    20
#define GIMBAL_AUTO_INDUSTRY_S_YAW_I 0.0f        
#define GIMBAL_AUTO_INDUSTRY_S_YAW_D 0.0f        //2


//*************打符PID
#define GIMBAL_AUTO_BUFF_P_PITCH_P 13.5f    //P自动位置环  110.0f   84.0    35.5      13.5
#define GIMBAL_AUTO_BUFF_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_BUFF_P_PITCH_D 0.0f    //200.0f                0

#define GIMBAL_AUTO_BUFF_S_PITCH_P 30.5f     //P自动速度环  3.5      3.8      14.2     30.1
#define GIMBAL_AUTO_BUFF_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_BUFF_S_PITCH_D 0.0f      

#define GIMBAL_AUTO_BUFF_P_YAW_P 25.3f       //Y自动位置环  80   54.3       25.3
#define GIMBAL_AUTO_BUFF_P_YAW_I 0.0f        
#define GIMBAL_AUTO_BUFF_P_YAW_D 0.0f        

#define GIMBAL_AUTO_BUFF_S_YAW_P 13.7f       //Y自动速度环  12   14.4       13.7
#define GIMBAL_AUTO_BUFF_S_YAW_I 0.0f        
#define GIMBAL_AUTO_BUFF_S_YAW_D 0.0f    
#elif (INFANTRY_NUM == INFANTRY_3)
/**********************Pitch轴PID参数**********************/
#define GIMBAL_P_PITCH_P  60.0f   //P位置环  70   
#define GIMBAL_P_PITCH_I  1.5f     // 1.1f
#define GIMBAL_P_PITCH_D  75.0f     //0.0f   75

#define GIMBAL_S_PITCH_P  10.5f   //P速度环(不要加i)       1.5     9.5
#define GIMBAL_S_PITCH_I  0.0f      
#define GIMBAL_S_PITCH_D  6.0f    //3.5  4
//#define GIMBAL_P_PITCH_P  600.0f   //P位置环    500
//#define GIMBAL_P_PITCH_I  20.0f     //  1.5f  2.0f
//#define GIMBAL_P_PITCH_D  400.0f     // 75

//#define GIMBAL_S_PITCH_P  20.5f   //P速度环(不要加i)        9.5
//#define GIMBAL_S_PITCH_I  0.1f      
//#define GIMBAL_S_PITCH_D  25.0f    //15


/**********************Yaw轴PID参数**********************/
#define GIMBAL_P_YAW_P  160.0f   //Y位置环    150     62    130
#define GIMBAL_P_YAW_I  0.0f       
#define GIMBAL_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_S_YAW_P  12.0f     //Y速度环     8      10     9
#define GIMBAL_S_YAW_I  0.0f   
#define GIMBAL_S_YAW_D  0.2f     //2                   0

#define GIMBAL_ROTATE_P_YAW_P  150.0f   //Y位置环    150     62    130
#define GIMBAL_ROTATE_P_YAW_I  0.0f       
#define GIMBAL_ROTATE_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_ROTATE_S_YAW_P  9.0f     //Y速度环     8      10     9
#define GIMBAL_ROTATE_S_YAW_I  0.0f   
#define GIMBAL_ROTATE_S_YAW_D  0.0f     //2                   0

/**********************低通滤波比例**********************/
#define Gimbal_Pitch_Fir_Ord_Low_Fil_Param  0.0884f

/**********************云台yaw角度限制**********************/
#define PITCH_ANGLE_LIMIT_UP    38
#define PITCH_ANGLE_LIMIT_DOWN  (-22)

/**********************运动加速度限制**********************/
#define GIMBAL_PITCH_ACCELERAD    2         //云台俯仰加速度限制
#define GIMBAL_YAW_ACCELERAD      2         //云台偏航加速度限制
#define GIMBAL_AUTO_YAW_ACCELERAD 1         //云台自动偏航加速度限制

/**********************pitch和yaw输出量限制**********************/
#define YAW_OUTPUT_LIMIT         11000 
#define YAW_INIT_OUTPUT_LIMIT    8000
#define PITCH_OUTPUT_LIMIT       14000   //  8000
#define PITCH_INIT_OUTPUT_LIMIT  5000

/**********************键盘鼠标遥控速度设置**********************/
#define MOUSE_YAW_SPEED       0.011f    //鼠标yaw轴速度增益     0.021f 
#define MOUSE_PITCH_SPEED     0.009f    //鼠标pitch轴速度增益  0.13
#define RC_YAW_SPEED          0.0026f   //遥控器yaw轴速度增益
#define RC_PITCH_SPEED        0.0026f   //遥控器pitch轴速度增益 0.0026



/*********视觉PY轴数据pid参数定义***************/

/**后面视觉需要三套pid，自瞄短焦，自瞄工业，打符工业**/

//*************短焦摄像头4mm
////P轴  x1.7f
//#define GIMBAL_AUTO_SHORT_P_PITCH_P 60.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_SHORT_P_PITCH_I 0.5f      
//#define GIMBAL_AUTO_SHORT_P_PITCH_D 70.2f      

//#define GIMBAL_AUTO_SHORT_S_PITCH_P 8.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_SHORT_S_PITCH_D 2.2f      //2
//P轴
#define GIMBAL_AUTO_SHORT_P_PITCH_P 15.5f     //P自动位置环   14
#define GIMBAL_AUTO_SHORT_P_PITCH_I 1.0f      
#define GIMBAL_AUTO_SHORT_P_PITCH_D 15.0f      // 40*40=1600   40.2f   15.0f

#define GIMBAL_AUTO_SHORT_S_PITCH_P 7.5f     //P自动速度环   13.5
#define GIMBAL_AUTO_SHORT_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_SHORT_S_PITCH_D 45.0f      //2   45.0f  
////上升
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_P 63.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_D 10.1f      

//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_P 10.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_D 0.0f  
////下降
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_P 22.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_D 9.1f      

//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_P 7.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_D 0.0f  
////上升
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_P 60.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_SHORT_P_PITCH_D 10.0f      

//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_P 8.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_SHORT_S_PITCH_D 0.0f  
////下降
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_P 21.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_SHORT_P_PITCH_D 20.1f      

//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_P 8.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_SHORT_S_PITCH_D 0.0f  
//Y轴
#define GIMBAL_AUTO_SHORT_P_YAW_P 170.0f       //Y自动位置环   160
#define GIMBAL_AUTO_SHORT_P_YAW_I 3.0f        // 2
#define GIMBAL_AUTO_SHORT_P_YAW_D 12.0f        //16

#define GIMBAL_AUTO_SHORT_S_YAW_P 24.6f       //Y自动速度环    7.6
#define GIMBAL_AUTO_SHORT_S_YAW_I 0.0f        
#define GIMBAL_AUTO_SHORT_S_YAW_D 0.0f        

//*************工业摄像头
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_P 50.0f    //P自动位置环  530.0f   35 50
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_I 2.0f      //3 
#define GIMBAL_AUTO_INDUSTRY_P_PITCH_D 0.0f    //200.0f    9.0    300

#define GIMBAL_AUTO_INDUSTRY_S_PITCH_P 8.0f     //P自动速度环  12  8
#define GIMBAL_AUTO_INDUSTRY_S_PITCH_I 0.0f      //0.02
#define GIMBAL_AUTO_INDUSTRY_S_PITCH_D 0.0f      //5  30
////上升
//#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_P 60.0f    //P自动位置环  530.0f   35
//#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

//#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_P 10.0f     //P自动速度环  12
//#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_D 0.0f      //5
////下降
//#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_P 26.0f    //P自动位置环  530.0f   35
//#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

//#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_P 8.0f     //P自动速度环  12
//#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_D 0.0f      //5
//yaw
#define GIMBAL_AUTO_INDUSTRY_P_YAW_P 160.0f       //Y自动位置环 170   130   100
#define GIMBAL_AUTO_INDUSTRY_P_YAW_I 2.0f        
#define GIMBAL_AUTO_INDUSTRY_P_YAW_D 15.0f        //15

#define GIMBAL_AUTO_INDUSTRY_S_YAW_P 20.0f       //Y自动速度环 12    20
#define GIMBAL_AUTO_INDUSTRY_S_YAW_I 0.0f        
#define GIMBAL_AUTO_INDUSTRY_S_YAW_D 0.0f        //2


//*************打符PID
#define GIMBAL_AUTO_BUFF_P_PITCH_P 13.5f    //P自动位置环  110.0f   84.0    35.5      13.5
#define GIMBAL_AUTO_BUFF_P_PITCH_I 0.0f      
#define GIMBAL_AUTO_BUFF_P_PITCH_D 0.0f    //200.0f                0

#define GIMBAL_AUTO_BUFF_S_PITCH_P 30.5f     //P自动速度环  3.5      3.8      14.2     30.1
#define GIMBAL_AUTO_BUFF_S_PITCH_I 0.0f      
#define GIMBAL_AUTO_BUFF_S_PITCH_D 0.0f      

#define GIMBAL_AUTO_BUFF_P_YAW_P 25.3f       //Y自动位置环  80   54.3       25.3
#define GIMBAL_AUTO_BUFF_P_YAW_I 0.0f        
#define GIMBAL_AUTO_BUFF_P_YAW_D 0.0f        

#define GIMBAL_AUTO_BUFF_S_YAW_P 13.7f       //Y自动速度环  12   14.4       13.7
#define GIMBAL_AUTO_BUFF_S_YAW_I 0.0f        
#define GIMBAL_AUTO_BUFF_S_YAW_D 0.0f    
///**********************Pitch轴PID参数**********************/
//#if (PITCH_PID_MODE == 1)
//	#define GIMBAL_UP_P_PITCH_P  41.5f   //P位置环  53   410    160       58
//	#define GIMBAL_UP_P_PITCH_I  0.0f     //8f      
//	#define GIMBAL_UP_P_PITCH_D  60.0f     //0.0f       360       100     100

//	#define GIMBAL_UP_S_PITCH_P  8.5f   //P速度环(不要加i)       1.5     9.5
//	#define GIMBAL_UP_S_PITCH_I  0.0f      
//	#define GIMBAL_UP_S_PITCH_D  3.0f   

//	#define GIMBAL_DOWN_P_PITCH_P  28.0f   //P位置环  53   410    160       58
//	#define GIMBAL_DOWN_P_PITCH_I  0.0f     //8f      
//	#define GIMBAL_DOWN_P_PITCH_D  200.0f     //0.0f       360       100     100

//	#define GIMBAL_DOWN_S_PITCH_P  7.5f   //P速度环(不要加i)       1.5     9.5
//	#define GIMBAL_DOWN_S_PITCH_I  0.0f   
//	#define GIMBAL_DOWN_S_PITCH_D  10.8f   
//#elif (PITCH_PID_MODE == 2)
//	#define GIMBAL_UP_P_PITCH_P  50.5f   //P位置环  53   410    160       58
//	#define GIMBAL_UP_P_PITCH_I  0.0f     //8f      
//	#define GIMBAL_UP_P_PITCH_D  60.0f     //0.0f       360       100     100

//	#define GIMBAL_UP_S_PITCH_P  10.5f   //P速度环(不要加i)       1.5     9.5
//	#define GIMBAL_UP_S_PITCH_I  0.0f      
//	#define GIMBAL_UP_S_PITCH_D  3.0f   

//	#define GIMBAL_DOWN_P_PITCH_P  30.0f   //P位置环  53   410    160       58
//	#define GIMBAL_DOWN_P_PITCH_I  0.0f     //8f      
//	#define GIMBAL_DOWN_P_PITCH_D  50.0f     //0.0f       360       100     100

//	#define GIMBAL_DOWN_S_PITCH_P  7.5f   //P速度环(不要加i)       1.5     9.5
//	#define GIMBAL_DOWN_S_PITCH_I  0.0f   
//	#define GIMBAL_DOWN_S_PITCH_D  20.8f   
//#elif (PITCH_PID_MODE == 3)
//	#define GIMBAL_UP_P_PITCH_P  41.5f   //P位置环  53   410    160       58
//	#define GIMBAL_UP_P_PITCH_I  0.0f     //8f      
//	#define GIMBAL_UP_P_PITCH_D  60.0f     //0.0f       360       100     100

//	#define GIMBAL_UP_S_PITCH_P  8.5f   //P速度环(不要加i)       1.5     9.5
//	#define GIMBAL_UP_S_PITCH_I  0.0f      
//	#define GIMBAL_UP_S_PITCH_D  3.0f   

//	#define GIMBAL_DOWN_P_PITCH_P  28.0f   //P位置环  53   410    160       58
//	#define GIMBAL_DOWN_P_PITCH_I  0.0f     //8f      
//	#define GIMBAL_DOWN_P_PITCH_D  200.0f     //0.0f       360       100     100

//	#define GIMBAL_DOWN_S_PITCH_P  7.5f   //P速度环(不要加i)       1.5     9.5
//	#define GIMBAL_DOWN_S_PITCH_I  0.0f   
//	#define GIMBAL_DOWN_S_PITCH_D  10.8f  
//#endif

///**********************Yaw轴PID参数**********************/
//#define GIMBAL_P_YAW_P  160.0f   //Y位置环    150     62    130
//#define GIMBAL_P_YAW_I  0.0f       
//#define GIMBAL_P_YAW_D  100.0f   //           0      100   100

//#define GIMBAL_S_YAW_P  12.0f     //Y速度环     8      10     9
//#define GIMBAL_S_YAW_I  0.0f   
//#define GIMBAL_S_YAW_D  0.2f     //2                   0

//#define GIMBAL_ROTATE_P_YAW_P  150.0f   //Y位置环    150     62    130
//#define GIMBAL_ROTATE_P_YAW_I  0.0f       
//#define GIMBAL_ROTATE_P_YAW_D  100.0f   //           0      100   100

//#define GIMBAL_ROTATE_S_YAW_P  9.0f     //Y速度环     8      10     9
//#define GIMBAL_ROTATE_S_YAW_I  0.0f   
//#define GIMBAL_ROTATE_S_YAW_D  0.0f     //2                   0

///**********************低通滤波比例**********************/
//#define Gimbal_Pitch_Fir_Ord_Low_Fil_Param  0.0884f

///**********************云台pitch和yaw角度限制**********************/
//#define PITCH_ANGLE_LIMIT_UP    38
//#define PITCH_ANGLE_LIMIT_DOWN  (-22)
//#define YAW_ANGLE_LIMIT         130

///**********************运动加速度限制**********************/
//#define GIMBAL_PITCH_ACCELERAD    2         //云台俯仰加速度限制
//#define GIMBAL_YAW_ACCELERAD      2         //云台偏航加速度限制
//#define GIMBAL_AUTO_YAW_ACCELERAD 1         //云台自动偏航加速度限制

///**********************pitch和yaw输出量限制**********************/
//#define YAW_OUTPUT_LIMIT         11000 
//#define YAW_INIT_OUTPUT_LIMIT    8000
//#define PITCH_OUTPUT_LIMIT       8000 
//#define PITCH_INIT_OUTPUT_LIMIT  5000

///**********************键盘鼠标遥控速度设置**********************/
//#define MOUSE_YAW_SPEED       0.011f    //鼠标yaw轴速度增益     0.021f 
//#define MOUSE_PITCH_SPEED     0.009f    //鼠标pitch轴速度增益  0.13
//#define RC_YAW_SPEED          0.0026f   //遥控器yaw轴速度增益
//#define RC_PITCH_SPEED        0.0026f   //遥控器pitch轴速度增益 0.0026



///*********视觉PY轴数据pid参数定义***************/

///**后面视觉需要三套pid，自瞄短焦，自瞄工业，打符工业**/

////短焦摄像头4mm
//#define GIMBAL_AUTO_SHORT_P_PITCH_P 30.5f     //P自动位置环   7.5
//#define GIMBAL_AUTO_SHORT_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_SHORT_P_PITCH_D 0.1f      

//#define GIMBAL_AUTO_SHORT_S_PITCH_P 5.5f     //P自动速度环   13.5
//#define GIMBAL_AUTO_SHORT_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_SHORT_S_PITCH_D 0.0f      

//#define GIMBAL_AUTO_SHORT_P_YAW_P 100.0f       //Y自动位置环   150
//#define GIMBAL_AUTO_SHORT_P_YAW_I 0.0f        
//#define GIMBAL_AUTO_SHORT_P_YAW_D 0.0f        

//#define GIMBAL_AUTO_SHORT_S_YAW_P 5.6f       //Y自动速度环    7.6
//#define GIMBAL_AUTO_SHORT_S_YAW_I 0.0f        
//#define GIMBAL_AUTO_SHORT_S_YAW_D 0.0f        

////工业摄像头
//#define GIMBAL_AUTO_INDUSTRY_P_PITCH_P 30.0f    //P自动位置环  530.0f   35
//#define GIMBAL_AUTO_INDUSTRY_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

//#define GIMBAL_AUTO_INDUSTRY_S_PITCH_P 8.0f     //P自动速度环  12
//#define GIMBAL_AUTO_INDUSTRY_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_INDUSTRY_S_PITCH_D 0.0f      //5
////上升
//#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_P 60.0f    //P自动位置环  530.0f   35
//#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

//#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_P 10.0f     //P自动速度环  12
//#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_UP_INDUSTRY_S_PITCH_D 0.0f      //5
////下降
//#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_P 26.0f    //P自动位置环  530.0f   35
//#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_INDUSTRY_P_PITCH_D 9.0f    //200.0f    9.0

//#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_P 8.0f     //P自动速度环  12
//#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_DOWN_INDUSTRY_S_PITCH_D 0.0f      //5
////yaw
//#define GIMBAL_AUTO_INDUSTRY_P_YAW_P 150.0f       //Y自动位置环 170   130   100
//#define GIMBAL_AUTO_INDUSTRY_P_YAW_I 0.0f        
//#define GIMBAL_AUTO_INDUSTRY_P_YAW_D 15.0f        //15

//#define GIMBAL_AUTO_INDUSTRY_S_YAW_P 20.0f       //Y自动速度环 12    20
//#define GIMBAL_AUTO_INDUSTRY_S_YAW_I 0.0f        
//#define GIMBAL_AUTO_INDUSTRY_S_YAW_D 0.0f        //2


////打符PID
//#define GIMBAL_AUTO_BUFF_P_PITCH_P 13.5f    //P自动位置环  110.0f   84.0    35.5      13.5
//#define GIMBAL_AUTO_BUFF_P_PITCH_I 0.0f      
//#define GIMBAL_AUTO_BUFF_P_PITCH_D 0.0f    //200.0f                0

//#define GIMBAL_AUTO_BUFF_S_PITCH_P 30.5f     //P自动速度环  3.5      3.8      14.2     30.1
//#define GIMBAL_AUTO_BUFF_S_PITCH_I 0.0f      
//#define GIMBAL_AUTO_BUFF_S_PITCH_D 0.0f      

//#define GIMBAL_AUTO_BUFF_P_YAW_P 25.3f       //Y自动位置环  80   54.3       25.3
//#define GIMBAL_AUTO_BUFF_P_YAW_I 0.0f        
//#define GIMBAL_AUTO_BUFF_P_YAW_D 0.0f        

//#define GIMBAL_AUTO_BUFF_S_YAW_P 13.7f       //Y自动速度环  12   14.4       13.7
//#define GIMBAL_AUTO_BUFF_S_YAW_I 0.0f        
//#define GIMBAL_AUTO_BUFF_S_YAW_D 0.0f 
#endif


typedef struct  //申明副云台电机变量
{
	const motor_measure_t *sec_gimbal_motor_measure;
	
    pid_parameter_t second_yaw_pid;  //pid

	int8_t init_flag;           //初始化成功标志
	int8_t photoelectric_zero;  //副Y轴中值光电标志
	
	int16_t Yaw_different_angle;       //副云台相对主云台的差角
	int16_t chassis_different_angle;   //副云台相对底盘的差角
	
	int16_t output;
	
} gimbal_second_control_t;

typedef struct  //申明pitch轴电机变量
{
	const motor_measure_t *pitch_motor_measure;
	
    pid_parameter_t pitch_p_pid;  //pid
	pid_parameter_t pitch_s_pid;  //pid

	pid_parameter_t pitch_auto_p_pid;  //pid
	pid_parameter_t pitch_auto_s_pid;  //pid
//	pid_parameter_t pitch_auto_up_p_pid;  //pid
//	pid_parameter_t pitch_auto_up_s_pid;  //pid
//	pid_parameter_t pitch_auto_down_p_pid;  //pid
//	pid_parameter_t pitch_auto_down_s_pid;  //pid
	
	float accel_up;
	float accel_down;
	
	int8_t init_flag;    //初始化成功标志
	
	float Auto_record_location;
	
	first_order_filter_type_t LowFilt_Pitch_Data;    //P轴低通滤波器
	sliding_mean_filter_type_t Slidmean_Pitch_Data;  //P轴滑动滤波器
	
	first_order_filter_type_t LowFilt_auto_pitch;    //自瞄P轴低通滤波器
	sliding_mean_filter_type_t Slidmean_auto_pitch;  //自瞄P轴滑动滤波器 

	int16_t filt_output; //P轴滤波值
	
	int16_t output;
	
} gimbal_pitch_control_t;

typedef struct  //申明yaw轴电机变量
{	
	motor_measure_t *yaw_motor_measure;

	pid_parameter_t yaw_p_pid;  //pid
	pid_parameter_t yaw_s_pid;  //pid
	
	pid_parameter_t yaw_auto_p_pid;  //pid
	pid_parameter_t yaw_auto_s_pid;  //pid
	
	pid_parameter_t yaw_rotate_p_pid;  //pid
	pid_parameter_t yaw_rotate_s_pid;  //pid
	
	bool_t init_flag;           //Y轴初始化成功标志
	int8_t photoelectric_zero;  //Y轴中值光电标志
	
//	int16_t chassis_different_angle;  //云台底盘差角
	
	float angle;              //云台当前绝对角度
	float last_angle;         //云台保存绝对角度
	float chassis_gimbal_angel;//云台和底盘差角

	int16_t filt_output;  //Y轴滤波值
	
	int16_t output;
	
} gimbal_yaw_control_t;


#define Finish 		 1
#define DisFinish  0

typedef struct  //申明抬升电机变量
{	
	const motor_measure_t *uplift_measure;

	pid_parameter_t uplift_p_pid;  //pid
	pid_parameter_t uplift_s_pid;  //pid
	
	float angle;              //云台当前绝对角度
	float last_angle;         //云台保存绝对角度

	uint8_t complete_state;				//状态
	
  fp32 speed;
  fp32 speed_set;
	int16_t output;
	
} uplift_control_t;

typedef struct  //申明平移电机变量
{	
	const motor_measure_t *translation_measure;

	pid_parameter_t translation_p_pid;  //pid
	pid_parameter_t translation_s_pid;  //pid

	fp32 speed;
  fp32 speed_set;
	float angle;              //云台当前绝对角度
	float last_angle;         //云台保存绝对角度

	uint8_t complete_state;				//状态
	
	int16_t output;
	
} translation_control_t;

typedef struct  //申明伸缩电机变量
{	
	const motor_measure_t *telescoping_measure;

	pid_parameter_t telescoping_p_pid;  //pid
	pid_parameter_t telescoping_s_pid;  //pid
	
  fp32 speed;
  fp32 speed_set;
	float angle;              //云台当前绝对角度
	float last_angle;         //云台保存绝对角度

	uint8_t complete_state;				//状态
	
	int16_t output;
	
} telescoping_control_t;

typedef struct  //申明夹紧电机变量
{	
	const motor_measure_t *clip_measure;

	pid_parameter_t clip_p_pid;  //pid
	pid_parameter_t clip_s_pid;  //pid
	
  fp32 speed;
  fp32 speed_set;
	float angle;              //云台当前绝对角度
	float last_angle;         //云台保存绝对角度
	
	uint8_t complete_state;				//状态
	
	int16_t output;
	
} clip_control_t;

typedef struct  //申明翻转电机变量
{	
	const motor_measure_t *flip_measure;

	pid_parameter_t flip_p_pid;  //pid
	pid_parameter_t flip_s_pid;  //pid

  fp32 speed;
  fp32 speed_set;
	
	float position_set;
	float angle_init;
	float angle;              //云台当前绝对角度
	float last_angle;         //云台保存绝对角度

	uint8_t complete_state;				//状态
	
	int16_t output;
	
} flip_control_t;

typedef struct
{
	const rc_ctrl_t *gimbal_RC; //底盘使用的遥控器指针
	Vision_Auto_Data_t *auto_c;    //申明自瞄变量
	const imu_data_t *imu_c;
	fire_task_t *fire_c;
	
	VL53L0_t *vL53L0;
	Sensor_t *sensor;
	
	uplift_control_t	uplift_c;
	translation_control_t	translation_c;
	telescoping_control_t	telescoping_c;
	clip_control_t clip_c;
	flip_control_t flip_c;
	
	
	gimbal_second_control_t sec_gimbal_c;  //申明副云台电机变量
	gimbal_pitch_control_t pitch_c;   //申明pitch轴电机变量
	gimbal_yaw_control_t yaw_c;       //申明yaw轴电机变量
	
	u8 Gimbal_all_flag;    //全部初始化完成标志
	u8 Gimbal_supply_flag; //补给状态标志位(0:归中状态，进入补给状态  1:云台90度转动中  2:到达指定位置  3:归中中   4：补给模式结束)
	
	u8 robo_state;
} gimbal_control_t;


/* 云台主任务 */
extern void gimbal_task(void *pvParameters);
/* 云台行为 */
extern void gimbal_manual_work(gimbal_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3);
extern void gimbal_automatic_work(gimbal_control_t *gimbal_automatic_work_f);
extern void gimbal_task_off(u8 options);

extern gimbal_control_t *get_gimbal_control_point(void);
void gimbal_change_pid(int x);

#endif
