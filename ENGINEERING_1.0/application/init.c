/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      ��ʼ����
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

/* �������ȼ���ֵԽС���������ȼ�Խ�� */

/* �������� */
#define Detect_TASK_PRIO 2
#define Detect_STK_SIZE 32
static TaskHandle_t DetectTask_Handler;

/* ��ȫ������ */
#define SAFE_TASK_PRIO 3
#define SAFE_STK_SIZE 128
static TaskHandle_t SafecheckTask_Handler;

/* ʧ�ؿ��Ƴ��� */
#define OUTCTL_Task_PRIO 2
#define OUTCTL_STK_SIZE 64
static TaskHandle_t OutCtlTask_Handler;

/* У׼���� */
#define CALIBRATE_TASK_PRIO 6
#define CALIBRATE_STK_SIZE 64
static TaskHandle_t CalibrateTask_Handler;

/* ui���� */
#define UI_TASK_PRIO 2
#define UI_STK_SIZE 64
static TaskHandle_t UiTask_Handler;

/* ��ʼ���� */
#define START_TASK_PRIO 6
#define START_STK_SIZE 128
static TaskHandle_t StartTask_Handler;

static TaskHandle_t *AppTask_Handler;
static TaskHandle_t *FireTask_Handler;

void hw_init(void);
uint8_t get_sys_cfg(void);

//�����ʼ��
void task_init(void)
{
    uint8_t app = 0;
    app = get_sys_cfg();
    if (app == CHASSIS_APP)
    {
        chassis_app_init(); // ����app��ʼ��
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
    xTaskCreate((TaskFunction_t)detect_task,          //һ����ͨ��������
                (const char *)"detect_task",          //��������
                (uint16_t)Detect_STK_SIZE,            //�����ջ��С
                (void *)NULL,                         //���ݸ��������Ĳ���
                (UBaseType_t)Detect_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&DetectTask_Handler); //������

    xTaskCreate((TaskFunction_t)safecheck_task, //������ȫ�������
                (const char *)"safecheck_task",
                (uint16_t)SAFE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)SAFE_TASK_PRIO,
                (TaskHandle_t *)&SafecheckTask_Handler);

    xTaskCreate((TaskFunction_t)outctl_task, //���� ʧ�ؿ��� ����
                (const char *)"outctl_task",
                (uint16_t)OUTCTL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)OUTCTL_Task_PRIO,
                (TaskHandle_t *)&OutCtlTask_Handler);
    vTaskSuspend(OutCtlTask_Handler); //�������

    xTaskCreate((TaskFunction_t)calibrate_task, //����У׼����
                (const char *)"calibrate_task",
                (uint16_t)CALIBRATE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)CALIBRATE_TASK_PRIO,
                (TaskHandle_t *)&CalibrateTask_Handler);
    vTaskSuspend(CalibrateTask_Handler); //�������
				
//    xTaskCreate((TaskFunction_t)ui_task, //����ui����
//                (const char *)"ui_task",
//                (uint16_t)UI_STK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)UI_TASK_PRIO,
//                (TaskHandle_t *)&UiTask_Handler);
}

/* ��ʼ���� */
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
  * @brief          �������������ʼ����
  * @param[in]      *argument
  * @retval         void
  */
void services_task(void const *argument)
{
    taskENTER_CRITICAL(); //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���
	
    hw_init();
	
    sys_task();
    task_init();

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	taskEXIT_CRITICAL();            //�˳��ٽ���
}

//ϵͳ����
void system_config(void)
{
    //glb_sys_cfg = HAL_GPIO_ReadPin(APP_CONFIG_GPIO_Port, APP_CONFIG_Pin); //ѡ��Ϊ ���̰� ���� ��̨��
}

//Ӳ�������ʼ��
void hw_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //����ϵͳ�ж����ȼ����� 4  xxxx 0000

    delay_init(180); //��ʱ������ʼ��

    /*GPIO��ʼ��*/
    led_init();

    /* CAN�ӿڳ�ʼ�� */
    can1_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);
    can2_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);

#ifdef WATCH_DOG
    /* ���Ź���ʼ�� */
    iwdg_init(IWDG_Prescaler_64, 300); //��������   4�� 300
#endif

    /*��ʼ�����״̬*/
    LEDB7 = 1;

    //system_config();  //�жϰ��ӵ�����
}

uint8_t get_sys_cfg(void)
{
    return glb_sys_cfg; //������ ���̰� ���� ��̨��
}

/**
  * @brief          ʧ��
  * @param[in]      none
  * @retval         none
  * @attention
  */
void out_of_control(void)
{
    //���������
    vTaskSuspend(DetectTask_Handler);
    vTaskSuspend(*AppTask_Handler);
#ifdef FIRE_WORK
    vTaskSuspend(*FireTask_Handler);
#endif

    //���ʧ�ر�����������
    vTaskResume(OutCtlTask_Handler);
}

/**
  * @brief          ����
  * @param[in]      none
  * @retval         none
  * @attention
  */
void normal_control(void)
{
    //�������
    vTaskResume(DetectTask_Handler);
    vTaskResume(*AppTask_Handler);
#ifdef FIRE_WORK
    vTaskResume(*FireTask_Handler);
#endif

    //ʧ�ر������������������
    vTaskSuspend(OutCtlTask_Handler);
}

/**
  * @brief          ����У׼����
  * @param[in]      none
  * @retval         none
  * @attention  
  */
void calibrate_task_hang(void)
{
    vTaskSuspend(CalibrateTask_Handler); //�������
}

/**
  * @brief          �������У׼����
  * @param[in]      none
  * @retval         none
  * @attention  
  */
void calibrate_task_remove_hang(void)
{
    vTaskResume(CalibrateTask_Handler); //�������
}
