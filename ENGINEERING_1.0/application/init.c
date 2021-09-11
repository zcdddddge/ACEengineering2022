/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      开始任务。
  ******************************************************************************
  */

#include "init.h"
#include "can.h"
#include "led.h"
#include "iwdg.h"
#include "ui_task.h"
#include "chassis_app.h"
#include "gimbal_app.h"
#include "shoot_task.h"
#include "security_task.h"
#include "detect_task.h"
#include "calibrate_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "parameter.h"

#pragma diag_suppress 177

#ifdef BOARD_CHASSIS
static const uint8_t glb_sys_cfg = CHASSIS_APP;
#endif
#ifdef BOARD_GIMBAL
static const uint8_t glb_sys_cfg = GIMBAL_APP;
#endif

/* 任务优先级数值越小，任务优先级越低 */

/* 心跳程序 */
#define Detect_TASK_PRIO 2
#define Detect_STK_SIZE 32
static TaskHandle_t DetectTask_Handler;

/* 安全监测程序 */
#define SAFE_TASK_PRIO 3
#define SAFE_STK_SIZE 128
static TaskHandle_t SafecheckTask_Handler;

/* 失控控制程序 */
#define OUTCTL_Task_PRIO 2
#define OUTCTL_STK_SIZE 64
static TaskHandle_t OutCtlTask_Handler;

/* 校准程序 */
#define CALIBRATE_TASK_PRIO 6
#define CALIBRATE_STK_SIZE 64
static TaskHandle_t CalibrateTask_Handler;

/* ui程序 */
#define UI_TASK_PRIO 2
#define UI_STK_SIZE 64
static TaskHandle_t UiTask_Handler;

/* 开始任务 */
#define START_TASK_PRIO 6
#define START_STK_SIZE 128
static TaskHandle_t StartTask_Handler;

static TaskHandle_t *AppTask_Handler;
static TaskHandle_t *FireTask_Handler;

void hw_init(void);
uint8_t get_sys_cfg(void);

//任务初始化
void task_init(void)
{
    uint8_t app = 0;
    app = get_sys_cfg();
    if (app == CHASSIS_APP)
    {
        chassis_app_init(); // 底盘app初始化
        AppTask_Handler = get_chassis_task_handle();
        //AppTask_Handler = xTaskGetHandle((const char *)"chassis_task");
    }
    else if (app == GIMBAL_APP)
    {
        gimbal_app_init();
        AppTask_Handler = get_gimbal_task_handle();
        //AppTask_Handler = xTaskGetHandle("gimbal_task");
    }
#ifdef FIRE_WORK
    shoot_app_init();
    FireTask_Handler = get_shoot_task_handle();
#endif
}

void sys_task(void)
{
    xTaskCreate((TaskFunction_t)detect_task,          //一个普通心跳程序
                (const char *)"detect_task",          //任务名称
                (uint16_t)Detect_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                         //传递给任务函数的参数
                (UBaseType_t)Detect_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&DetectTask_Handler); //任务句柄

    xTaskCreate((TaskFunction_t)safecheck_task, //创建安全监测任务
                (const char *)"safecheck_task",
                (uint16_t)SAFE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)SAFE_TASK_PRIO,
                (TaskHandle_t *)&SafecheckTask_Handler);

    xTaskCreate((TaskFunction_t)outctl_task, //创建 失控控制 任务
                (const char *)"outctl_task",
                (uint16_t)OUTCTL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)OUTCTL_Task_PRIO,
                (TaskHandle_t *)&OutCtlTask_Handler);
    vTaskSuspend(OutCtlTask_Handler); //任务挂起

    xTaskCreate((TaskFunction_t)calibrate_task, //创建校准任务
                (const char *)"calibrate_task",
                (uint16_t)CALIBRATE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CALIBRATE_TASK_PRIO,
                (TaskHandle_t *)&CalibrateTask_Handler);
    vTaskSuspend(CalibrateTask_Handler); //任务挂起
				
//    xTaskCreate((TaskFunction_t)ui_task, //创建ui任务
//                (const char *)"ui_task",
//                (uint16_t)UI_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)UI_TASK_PRIO,
//                (TaskHandle_t *)&UiTask_Handler);
}

/* 开始任务 */
void start_task(void)
{
    xTaskCreate((TaskFunction_t)services_task,
                (const char *)"services_task",
                (uint16_t)START_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIO,
                (TaskHandle_t *)&StartTask_Handler);
}

/**
  * @brief          所有任务、外设初始化。
  * @param[in]      *argument
  * @retval         void
  */
void services_task(void const *argument)
{
    taskENTER_CRITICAL(); //为了保证对PORTA寄存器的访问不被中断，将访问操作放入临界区。进入临界区
	
    hw_init();
	
    sys_task();
    task_init();

    vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}

//系统配置
void system_config(void)
{
    //glb_sys_cfg = HAL_GPIO_ReadPin(APP_CONFIG_GPIO_Port, APP_CONFIG_Pin); //选择为 底盘板 还是 云台板
}

//硬件外设初始化
void hw_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //设置系统中断优先级分组 4  xxxx 0000

    delay_init(180); //延时函数初始化

    /*GPIO初始化*/
    led_init();

    /* CAN接口初始化 */
    can1_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);
    can2_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);

#ifdef WATCH_DOG
    /* 看门狗初始化 */
    iwdg_init(IWDG_Prescaler_64, 300); //半秒多溢出   4， 300
#endif

    /*初始化完成状态*/
    LEDB7 = 1;

    //system_config();  //判断板子的类型
}

uint8_t get_sys_cfg(void)
{
    return glb_sys_cfg; //返回是 底盘板 还是 云台板
}

/**
  * @brief          失控
  * @param[in]      none
  * @retval         none
  * @attention
  */
void out_of_control(void)
{
    //将任务挂起
    vTaskSuspend(DetectTask_Handler);
    vTaskSuspend(*AppTask_Handler);
#ifdef FIRE_WORK
    vTaskSuspend(*FireTask_Handler);
#endif

    //解挂失控保护控制任务
    vTaskResume(OutCtlTask_Handler);
}

/**
  * @brief          正常
  * @param[in]      none
  * @retval         none
  * @attention
  */
void normal_control(void)
{
    //解挂任务
    vTaskResume(DetectTask_Handler);
    vTaskResume(*AppTask_Handler);
#ifdef FIRE_WORK
    vTaskResume(*FireTask_Handler);
#endif

    //失控保护控制任务任务挂起
    vTaskSuspend(OutCtlTask_Handler);
}

/**
  * @brief          挂起校准任务
  * @param[in]      none
  * @retval         none
  * @attention  
  */
void calibrate_task_hang(void)
{
    vTaskSuspend(CalibrateTask_Handler); //任务挂起
}

/**
  * @brief          解除挂起校准任务
  * @param[in]      none
  * @retval         none
  * @attention  
  */
void calibrate_task_remove_hang(void)
{
    vTaskResume(CalibrateTask_Handler); //解挂任务
}
