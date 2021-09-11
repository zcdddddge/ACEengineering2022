#include "double_gimbal.h"

Second_Gimbal_Data_Typedef second_gimbal = {0};

/*
*���ܣ�����̨PY����ƴ���
*���룺����̨PY����ʵ������
*�����ֱ�Ӹ�PWMֵ�����������
*/
//PA0Ϊ��P��ת����ƿ�   PA1Ϊ��y��ת����ƿ�
//�ߵ�ƽ��ת���͵�ƽ˳ת
//P����TIM5 CCR3  PH12
//Y����TIM5 CCR4  PI0
void Second_Gimbal_Control(int16_t yaw_control_data, int16_t pitch_contral_data)
{
    /*yaw�����*/
    if (yaw_control_data > 0) //˳ת
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        TIM5->CCR4 = yaw_control_data;
    }
    else if (yaw_control_data < 0) //��ת
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        TIM5->CCR4 = (-yaw_control_data);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        TIM5->CCR4 = 0;
    }

    /*pitch�����*/
    if (pitch_contral_data > 0) //˳ת
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        TIM5->CCR3 = pitch_contral_data;
    }
    else if (pitch_contral_data < 0) //��ת
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        TIM5->CCR3 = (-pitch_contral_data);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        TIM5->CCR3 = 0;
    }
}
