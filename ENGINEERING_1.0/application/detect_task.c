#include "detect_task.h"
#include "led.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "upper_machine.h"
#include "init.h"
#include "usart6.h"
#include "uart4.h"
#include "referee_deal.h"

#pragma diag_suppress 177

#define ERROR_LIST_LENGHT 9  //������̨�������������Ǹ�


error_t error_list[ERROR_LIST_LENGHT+1];
uint8_t error_num_display = 0; //static 

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif

static void chassis_detect_init(uint32_t time);
static void gimbal_detect_init(uint32_t time);
extern uint8_t get_sys_cfg(void);
void detect_task(void *pvParameters)
{
	uint8_t usart6_dma_test[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
	static uint32_t App_list_len = 0;
	static uint32_t system_time;
//    system_time = xTaskGetTickCount();
//	
//    WORK_HEART = 0;
//	
//    uint8_t app = get_sys_cfg();
//    if (app == CHASSIS_APP)
//    {
//        chassis_detect_init(system_time);
//		App_list_len = CHASSIS_ERROR_LIST_LENGHT;
//    }
//    else if (app == GIMBAL_APP)
//    {
//        gimbal_detect_init(system_time);
//		App_list_len = GIMBAL_ERROR_LIST_LENGHT;
//    }

    //����һ��ʱ��
    vTaskDelay(5);

    while (1)
    {
		WORK_HEART = !WORK_HEART;
		vTaskDelay(100);  //100
//		//vofa_test();
//		//pid_parameter_receive(NULL, NULL);
		
//		referee_read_data();
//		uart4_dma_send(usart6_dma_test, 10);
		
		
//        system_time = xTaskGetTickCount();

//        error_num_display = ERROR_LIST_LENGHT;
//        error_list[ERROR_LIST_LENGHT].is_lost = 0;

//        for (int i = 0; i < App_list_len; i++)
//        {
//            //! δʹ�ܣ�����
//            if (error_list[i].enable == 0)
//            {
//                continue;
//            }

//            //! judge offline.�жϵ���
//            if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
//            {
//				//��¼�����Լ�����ʱ��
//				error_list[i].is_lost = 1;
//				error_list[i].lost_time = system_time;
//				
//                //�жϴ������ȼ��� �������ȼ���ߵĴ�����
//                if (error_list[i].priority > error_list[error_num_display].priority)
//                {
//                    error_num_display = i;
//                }

//                error_list[ERROR_LIST_LENGHT].is_lost = 1;
//				
//                //����ṩ������������н������
//                if (error_list[i].solve_lost_fun != NULL)
//                {
//                    error_list[i].solve_lost_fun();
//                }
//            }
//            else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
//            {
//                //�ո����ߣ����ܴ������ݲ��ȶ���ֻ��¼����ʧ��
//                error_list[i].is_lost = 0;
//            }
//            else
//            {
//                error_list[i].is_lost = 0;
//				
//                //����Ƶ��
//                if (error_list[i].new_time > error_list[i].last_time)
//                {
//                    error_list[i].frequency = configTICK_RATE_HZ / (fp32)(error_list[i].new_time - error_list[i].last_time);
//                }
//            }
//        }

//        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



/**
  * @brief          ��¼ʱ��
  * @param[in]      toe:�豸Ŀ¼
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();

    if (error_list[toe].is_lost)
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }
}


void chassis_detect_init(uint32_t time)
{
    //��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ� offlineTime onlinetime priority
    uint16_t set_item[CHASSIS_ERROR_LIST_LENGHT][3] =
        {
            {30, 40, 9},  //usart1_rc
            {10, 10, 8},  //motor1
            {10, 10, 7},  //motor2
            {10, 10, 6},   //motor3
            {10, 10, 5},   //motor4
			{10, 10, 4},   //Supercapacitor
			{10, 10, 3},   //gimbal
            {10, 10, 2},  //trigger
            {100, 100, 1}, //referee
        };
    for (uint8_t i = 0; i < CHASSIS_ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];  //��������ʱ��
        error_list[i].set_online_time = set_item[i][1];   //��������ʱ��
        error_list[i].priority = set_item[i][2];          //��������
        error_list[i].solve_lost_fun = NULL;              //���ߴ�����

        error_list[i].enable = 1;        //�Ƿ�ʹ��
        error_list[i].is_lost = 1;       //�Ƿ�����
        error_list[i].frequency = 0.0f;  //Ƶ��
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }
}

void gimbal_detect_init(uint32_t time)
{
    //��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ� offlineTime onlinetime priority
    uint16_t set_item[GIMBAL_ERROR_LIST_LENGHT][3] =
        {
            {30, 40, 6},  //can2_rc
            {2, 3, 5},    //yaw
            {2, 3, 4},    //pitch
			{10, 10, 3},   //chassis
			{10, 10, 2},   //visual
            {10, 10, 1},   //imu
        };
    for (uint8_t i = 0; i < GIMBAL_ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];  //��������ʱ��
        error_list[i].set_online_time = set_item[i][1];   //��������ʱ��
        error_list[i].priority = set_item[i][2];          //��������
        error_list[i].solve_lost_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].is_lost = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }
}





/**
  * @brief          ���Լ���ϵͳʣ����
  * @param[in]      none
  * @retval         none
  * @attention      FreeRTOSConfig.h �е����ó��� configUSE_IDLE_HOOK ���붨��Ϊ 1���������������Ӻ����Żᱻ���á�
  *                 ���й��Ӻ�����������ΪvApplicationIdleHook(),�޲���Ҳ�޷���ֵ��
  */
void vApplicationIdleHook(void)
{
//  ulIdleCycleCount = xTaskGetTickCount(); //��ȡ��ǰϵͳʱ��
}


