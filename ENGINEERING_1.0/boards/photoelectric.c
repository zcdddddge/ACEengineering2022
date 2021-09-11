#include "photoelectric.h"

/*����̨yaw����ֵ���*/
void yaw_zero_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*����̨yaw����ֵ���*/
void yaw_two_zero_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*
* ���ܣ��������̨Y����ֵ��緵��ֵ
* ���룺��
* �������緵��ֵ1��0������ֵ����1������ֵ����0��
*/
uint8_t yaw_zero_value(void) //���Y���Ƿ�λ   ����ֵ����0�����򷵻�1
{
    return (!(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)));
}

/*
* ���ܣ���⸱��̨Y����ֵ��緵��ֵ
* ���룺��
* �������緵��ֵ1��0������ֵ����1������ֵ����0��
*/
uint8_t sec_yaw_zero_value(void) //���Y���Ƿ�λ   ����ֵ����0�����򷵻�1
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
}

/*===============================Yaw��������λ�õĹ��У��===============================*/
static int16_t correctYawValue[5] = {0}; //��⵽����0
void correct_yaw_zero(gimbal_control_t *fir_gimbal_correct)
{
    uint8_t i = 0;
    u16 correctSum = 0;

    /* ʵʱ���YawУ׼���  (����ֵ��0) */
    fir_gimbal_correct->yaw_c.photoelectric_zero = yaw_zero_value();

    for (i = 0; i < 5; i++)
    {
        correctYawValue[i - 1] = correctYawValue[i];
    }
    correctYawValue[4] = fir_gimbal_correct->yaw_c.photoelectric_zero; //

    for (i = 0; i < 5; i++)
    {
        correctSum += correctYawValue[i];
    }

    correctSum /= 5;

    if (correctSum == 0)
    {
        fir_gimbal_correct->yaw_c.yaw_motor_measure->actual_Position = 0;
    }
}

///*===============================��Yaw��������λ�õĹ��У��===============================*/
//static int16_t correctSecYawValue[5] = {0};
//void Correct_Sec_Yaw_Zero(gimbal_second_control_t *sec_gimbal_correct)
//{
//    uint8_t i = 0;
//    u16 correctSum = 0;

//    for (i = 0; i < 5; i++)
//    {
//        correctSecYawValue[i - 1] = correctSecYawValue[i];
//    }
//    correctSecYawValue[4] = !sec_gimbal_correct->photoelectric_zero; //Yaw_Zero_Value();

//    for (i = 0; i < 5; i++)
//    {
//        correctSum += correctSecYawValue[i];
//    }

//    correctSum /= 5;

//    if (correctSum == 0)
//    {
//        motor_second_yaw.actual_Position = 0;
//    }
//}
