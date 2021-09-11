#include "chassis_app.h"
#include "delay.h"
#include "can_1_receive.h"
#include "can_2_receive.h"
#include "referee_deal.h"
#include "capacitor_control.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
#include "uart4.h"
#include "usart6.h"
#include "rc.h"
#include "timer_task.h"
#include "security_task.h"
#include "upper_machine.h"

/* 底盘控制任务 */
#define Chassis_TASK_PRIO 5
#define Chassis_STK_SIZE 128
static TaskHandle_t ChassisTask_Handler;

TaskHandle_t *get_chassis_task_handle(void)
{
    return &ChassisTask_Handler;
}

/**
  * @brief      chassis app init
  * @param[in]  
  * @retval     void
*/
void chassis_app_init(void)
{
    /* 串口初始化 */
    remote_control_init();         //串口一初始化（遥控器初始化）
    upper_machine_communication(); //上位机debug
		UART4_GYRO_INIT_PC11_RX_PC10_TX();
//    referee_system_init();         //串口六初始化（裁判系统）	

    can1_callback = chassis_can1_callback;
    can2_callback = chassis_can2_callback;
    rc_lost_time = chassis_rc_lost_time;
    rc_callback = chassis_rc_callback;
		usart6_callback = NULL;

    delay_ms(1000);
	
	//创建软件定时器
    timer_send_create(); 
	timer_receive_create(); 

    //完成底盘控制任务
    xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"chassis_task",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
}

/* 底盘板CAN1发送到底盘电机 */
void can1_chassis_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
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

/* 底盘板CAN1发送到救援电机 */
void can1_rescue_setmsg(int16_t ESC_205, int16_t ESC_206)
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
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_206 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_206);
    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}
/* 底盘板CAN1发送到小云台p电机 */
void can1_spitch_setmsg(int16_t ESC_208)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x1ff;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            // 发送两帧信息

    CAN1_TxMessage.Data[6] = (unsigned char)(ESC_208 >> 8);
    CAN1_TxMessage.Data[7] = (unsigned char)(ESC_208);
    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}
/* 底盘板CAN1发送到小云台y电机 */
void can1_syaw_setmsg(int16_t ESC_207)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x2FF;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            // 发送两帧信息

		CAN1_TxMessage.Data[4] = (unsigned char)ESC_207>>8;
		CAN1_TxMessage.Data[5] = (unsigned char)ESC_207;
    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}


/* 底盘板CAN1发送到云台Y电机和供弹电机 | Y轴电机是205，P轴电机是206，供弹电机是207（P轴电机0x206没用到） */
void can1_chassis_gimbal_fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x1ff;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            //指定数据的长度
    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_205 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_205);
    CAN1_TxMessage.Data[2] = 0;
    CAN1_TxMessage.Data[3] = 0;
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_207 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_207);
    CAN1_TxMessage.Data[6] = 0;
    CAN1_TxMessage.Data[7] = 0;
    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束
}

//买的电容
//设定功率
void can1_cap_setmsg(int16_t Chassis_power)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    Chassis_power = Chassis_power * 100;

    CAN1_TxMessage.StdId = 0x210;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 2;            // 发送两帧信息

    CAN1_TxMessage.Data[0] = (uint8_t)(Chassis_power >> 8);
    CAN1_TxMessage.Data[1] = (uint8_t)(Chassis_power);

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}

/* 底盘板CAN2遥控器数据发送 */
void can2_chassis_rc_setmsg(const rc_ctrl_t *can2_RC_send)
{
    u8 mbox;
    u16 i = 0;

    CanTxMsg CAN2_TxMessage; //定义一个发送信息的结构体

    CAN2_TxMessage.StdId = 0x400;
    CAN2_TxMessage.IDE = CAN_ID_STD;
    CAN2_TxMessage.RTR = CAN_RTR_DATA;
    CAN2_TxMessage.DLC = 8;

    CAN2_TxMessage.Data[0] = (unsigned char)(can2_RC_send->rc.ch[2] >> 8);
    CAN2_TxMessage.Data[1] = (unsigned char)(can2_RC_send->rc.ch[2]);
    CAN2_TxMessage.Data[2] = (unsigned char)(can2_RC_send->rc.ch[3] >> 8);
    CAN2_TxMessage.Data[3] = (unsigned char)(can2_RC_send->rc.ch[3]);
    CAN2_TxMessage.Data[4] = (unsigned char)(can2_RC_send->rc.ch[4] >> 8);
    CAN2_TxMessage.Data[5] = (unsigned char)(can2_RC_send->rc.ch[4]);
    CAN2_TxMessage.Data[6] = (unsigned char)((can2_RC_send->rc.s1) * 10 + (can2_RC_send->rc.s2));
    CAN2_TxMessage.Data[7] = (unsigned char)((can2_RC_send->mouse.press_l + 1) * 10 + can2_RC_send->mouse.press_r);

    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}

/* 底盘板CAN2键盘数据发送 */
void can2_chassis_mk_setmsg(const rc_ctrl_t *can2_MK_send)
{
    u16 i = 0;
    u8 mbox;
    CanTxMsg TxMessage;

    TxMessage.StdId = 0x401;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;

    TxMessage.Data[0] = (unsigned char)(can2_MK_send->mouse.x >> 8);
    TxMessage.Data[1] = (unsigned char)(can2_MK_send->mouse.x);
    TxMessage.Data[2] = (unsigned char)(can2_MK_send->mouse.y >> 8);
    TxMessage.Data[3] = (unsigned char)(can2_MK_send->mouse.y);
    TxMessage.Data[4] = (unsigned char)(can2_MK_send->kb.key_code >> 8);
    TxMessage.Data[5] = (unsigned char)(can2_MK_send->kb.key_code);
    TxMessage.Data[6] = (unsigned char)((can2_MK_send->rc.s1) * 10 + (can2_MK_send->rc.s2));
    TxMessage.Data[7] = (unsigned char)((can2_MK_send->mouse.press_l + 1) * 10 + can2_MK_send->mouse.press_r);

    mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}

/* 底盘板CAN2发送 */
void can2_4002_send(void)
{
    u8 mbox;
    u16 i = 0;
    uint16_t speed_max_limit = referee_shooter_id1_17mm_speed_limit(); //机器人 1 号 17mm 枪口上限速度 单位 m/s
    CanTxMsg TxMessage;

    TxMessage.StdId = 0x402;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.Data[0] = (uint8_t)(speed_max_limit >> 8);
    TxMessage.Data[1] = (uint8_t)(speed_max_limit);
    TxMessage.Data[2] = (uint8_t)(0);
    TxMessage.Data[3] = (uint8_t)(0);
    TxMessage.Data[4] = (uint8_t)(0);
    TxMessage.Data[5] = (uint8_t)(0);
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;

    mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}

/* 测试 */
void can2_gimbal_setmsg_2(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x1ff;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;

    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_205 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_205);
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_206 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_206);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_207 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_207);
    CAN1_TxMessage.Data[6] = (unsigned char)(ESC_208 >> 8);
    CAN1_TxMessage.Data[7] = (unsigned char)(ESC_208);

    mbox = CAN_Transmit(CAN2, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}

/* 测试 */
void can2_chassis_setmsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
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

    mbox = CAN_Transmit(CAN2, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}
