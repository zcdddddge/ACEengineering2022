#ifndef __TASK_DETECT_H
#define __TASK_DETECT_H

#include "struct_typedef.h"

//#define DETECT_TASK_INIT_TIME  5
//#define DETECT_CONTROL_TIME    1000
#define DETECT_TASK_INIT_TIME  57
#define DETECT_CONTROL_TIME    10




//错误码以及对应设备顺序
enum chassis_errorList
{
   USART_RC_TOE = 0,
   CHASSIS_MOTOR1_TOE,
   CHASSIS_MOTOR2_TOE,
   CHASSIS_MOTOR3_TOE,
   CHASSIS_MOTOR4_TOE,
   SUPER_CAPACITOR_TOE,
   GIMBAL_TOE,
   TRIGGER_MOTOR_TOE,
   REFEREE_TOE,
   CHASSIS_ERROR_LIST_LENGHT,
};
//错误码以及对应设备顺序
enum gimbal_errorList
{
   CAN2_RC_TOE = 0,
   YAW_GIMBAL_MOTOR_TOE,
   PITCH_GIMBAL_MOTOR_TOE,
   CHASSIS_TOE,
   VISUAL_TOE,
   IMU_TOE,
   GIMBAL_ERROR_LIST_LENGHT,
};

//#if (CHASSIS_ERROR_LIST_LENGHT == 0)
//#define ERROR_LIST_LENGHT CHASSIS_ERROR_LIST_LENGHT  //底盘云台，两个中最大的那个
//#else
//#define ERROR_LIST_LENGHT GIMBAL_ERROR_LIST_LENGHT
//#endif

typedef struct
{
    uint32_t new_time;
    uint32_t last_time;
    uint32_t lost_time;
    uint32_t work_time;
    uint16_t set_offline_time;
    uint16_t set_online_time;
    uint8_t enable;
    uint8_t priority;
    uint8_t is_lost;

    fp32 frequency;     //频率
    void (*solve_lost_fun)(void);  //离线解决函数
} error_t;




void detect_task(void *pvParameters);

#endif
