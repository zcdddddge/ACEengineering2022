#include "gimbal_app.h"
#include "delay.h"
#include "can_1_receive.h"
#include "can_2_receive.h"
#include "rc.h"
#include "key.h"
#include "imu.h"
#include "photoelectric.h"
#include "led.h"
#include "pwm.h"
#include "security_task.h"
#include "upper_machine.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
#include "uart4.h"
#include "usart6.h"

/* 云台控制任务 */
#define GIMBAL_TASK_PRIO 5
#define GIMBAL_STK_SIZE 256
static TaskHandle_t GimbalTask_Handler;

TaskHandle_t *get_gimbal_task_handle(void)
{
    return &GimbalTask_Handler;
}

/**
  * @brief  chassis app init
  * @param
  * @retval void
  */
void gimbal_app_init(void)
{
    /* 串口初始化 */
    imu_control_init();            //串口一初始化（接收陀螺仪数据）
    upper_machine_communication(); //上位机debug
    automatic_aiming_init();       //串口三初始化（视觉通讯）

    /* 光电初始化 */
    yaw_zero_gpio_init();     //y轴中值光电初始化 pa3
    yaw_two_zero_gpio_init(); //副y轴中值光电初始化 pa2

    /*pwm初始化*/
    friction_wheel_init(); //摩擦轮初始化

#ifdef FIRE_WORK
    laser_init(); //激光初始化
    laser_on();   //打开激光
#endif

    can1_callback = gimbal_can1_callback;
    can2_callback = gimbal_can2_callback;
		usart6_init(5);
		sensor_init();
    rc_callback = NULL;
//    usart6_callback = NULL;

    rc_lost_time = gimbal_rc_lost_time;

    vTaskDelay(2000);
	vTaskDelay(2000);
	vTaskDelay(2000);

    //完成云台控制任务
    xTaskCreate((TaskFunction_t)gimbal_task,
                (const char *)"gimbal_task",          //任务名称
                (uint16_t)GIMBAL_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                         //传递给任务函数的参数
                (UBaseType_t)GIMBAL_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&GimbalTask_Handler); //任务句柄
}

/* 云台板CAN1发送到P轴电机 | P轴电机是201，副云台yaw2006是202 */
void can1_gimbal_setmsg(int16_t ESC_201, int16_t ESC_202)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x200;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            //指定数据的长度
    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_201 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_201);
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_202);
    CAN1_TxMessage.Data[4] = 0;
    CAN1_TxMessage.Data[5] = 0;
    CAN1_TxMessage.Data[6] = 0;
    CAN1_TxMessage.Data[7] = 0;

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束
}

void can1_function_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x200;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            // 发送两帧信息

    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_201 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_201);
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_202);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_203 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_203);
    CAN1_TxMessage.Data[6] = (unsigned char)(ESC_204 >> 8);
    CAN1_TxMessage.Data[7] = (unsigned char)(ESC_204);

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}

void can1_flip_setmsg(int16_t ESC_205)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x1ff;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            // 发送两帧信息

    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_205 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_205);
    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}



/* 云台板CAN2发送 */
void can2_gimbal_setmsg(u8 Initialize_flag, u8 Yaw_zero_flag, u8 Supply_flag, int16_t gimbal_dif_angel, uint8_t gimbal_beh, uint16_t pitch_angle)
{
    u8 mbox;
    u16 i = 0;

    CanTxMsg TxMessage;

    TxMessage.StdId = 0x300;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 6;

    TxMessage.Data[0] = (uint8_t)((Initialize_flag)*100 + Yaw_zero_flag * 10 + Supply_flag);
    TxMessage.Data[1] = (uint8_t)(gimbal_dif_angel >> 8);
    TxMessage.Data[2] = (uint8_t)(gimbal_dif_angel);
    TxMessage.Data[3] = (uint8_t)(gimbal_beh);
    TxMessage.Data[4] = (uint8_t)(pitch_angle >> 8);
    TxMessage.Data[5] = (uint8_t)(pitch_angle);
    //	TxMessage.Data[6] = (uint8_t)();

    mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}

/* 云台板CAN2发送 */
void can2_sensor_setmsg(u8 sensor_l, u8 sensor_r)
{
    u8 mbox;
    u16 i = 0;

    CanTxMsg TxMessage;

    TxMessage.StdId = 0x301;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 6;

    TxMessage.Data[0] = (uint8_t)(sensor_l);
    TxMessage.Data[1] = (uint8_t)(sensor_r);
    //	TxMessage.Data[6] = (uint8_t)();

    mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}



void can2_yaw_setmsg(int16_t gimbal_output)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN2_TxMessage; //定义一个发送信息的结构体

    CAN2_TxMessage.StdId = 0x1ff;      //根据820r设置标识符
    CAN2_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN2_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN2_TxMessage.DLC = 2;            // 发送两帧信息

    CAN2_TxMessage.Data[0] = (uint8_t)(gimbal_output >> 8);
    CAN2_TxMessage.Data[1] = (uint8_t)(gimbal_output);
    CAN2_TxMessage.Data[2] = 0;
    CAN2_TxMessage.Data[3] = 0;
    CAN2_TxMessage.Data[4] = 0;
    CAN2_TxMessage.Data[5] = 0;
    CAN2_TxMessage.Data[6] = 0;
    CAN2_TxMessage.Data[7] = 0;

    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}
