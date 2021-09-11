#include "calibrate_task.h"
#include "imu.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "rm_motor.h"
#include "init.h"
#include "can_2_receive.h"

//��������̨����
static gimbal_control_t *calibration_c;
static uint8_t imu_zero_calibration = 0;
static uint8_t Friction_wheel_initialized_flag = 0;

//��̨��ʼ��
static void Gimbal_Location_Init(gimbal_control_t *fir_gimbal_location_init_f);   
//Yaw���ʼ��
//static void Yaw_Init(gimbal_control_t *yaw_location_init_f);
//Pitch���ʼ��
//static void Pitch_Init(gimbal_control_t *pitch_location_init_f);  



void calibrate_task(void *pvParameters)
{
	vTaskDelay(5);    //����һ��ʱ��
	
	calibration_c = get_gimbal_control_point();
		
    while (1)
    {		
		Gimbal_Location_Init(calibration_c);
		
		vTaskDelay(2);
    }
}



/**
  * @brief          ��̨��ʼ��
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Gimbal_Location_Init(gimbal_control_t *fir_gimbal_location_init_f)
{
	//��������ʼ���Ļ�ֱ�Ӽ��Ͼͺ��ˣ����õ�ʱ��ֱ��ע�͵�
	fir_gimbal_location_init_f->yaw_c.init_flag = 1;
	fir_gimbal_location_init_f->pitch_c.init_flag = 1;
	fir_gimbal_location_init_f->sec_gimbal_c.init_flag = 1;
	imu_zero_calibration = 0;
	
//	Pitch_Init(fir_gimbal_location_init_f);   //P���ʼ��
//	Yaw_Init(fir_gimbal_location_init_f);     //Y���ʼ��
//	Sec_Yaw_Init(); //��Y���ʼ��
	
//	delay_ms(500);
//	imu_zero_correct();
//	delay_ms(2000);
	imu_zero_calibration = 1;
	

//	TIM4->CCR1 = 1000;
//	TIM4->CCR2 = 1000;
	vTaskDelay(2000);
	Friction_wheel_initialized_flag = 1;
	vTaskDelay(2000);
	
	//����������ʼ���ɹ�
    if (fir_gimbal_location_init_f->yaw_c.init_flag == 1 && 
		fir_gimbal_location_init_f->pitch_c.init_flag == 1 && 
		fir_gimbal_location_init_f->sec_gimbal_c.init_flag == 1 && 
		imu_zero_calibration == 1 &&
		Friction_wheel_initialized_flag == 1)
    {
        fir_gimbal_location_init_f->yaw_c.init_flag = 0;
		fir_gimbal_location_init_f->pitch_c.init_flag = 0;
		fir_gimbal_location_init_f->sec_gimbal_c.init_flag = 0;
		fir_gimbal_location_init_f->Gimbal_all_flag = 1;
		
		calibrate_task_hang();  //����У׼����
    }
}


