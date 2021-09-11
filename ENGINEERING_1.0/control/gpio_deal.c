#include "gpio_deal.h"
#include "key.h"

/*GPIOx*/
#define GPIO_GRASP_1 GPIOA
#define GPIO_GRASP_2 GPIOB
#define GPIO_GRASP_3 GPIOC


static const uint16_t Smooth_L_PIN = GPIO_Pin_7;
static const uint16_t Smooth_R_PIN = GPIO_Pin_6;
//static const uint16_t Stretch_PIN 	= GPIO_Pin_6;

/*��Ч��������*/
static const u8 FilteNum = 2;

Sensor_t Sensor;

/*���ش�������ֵ�ṹ��*/
Sensor_t *Return_Sensor_t(void)
{
    return &Sensor;
}

/*ƽ���󴫸���*/
u8 Return_Smooth_L_Val(void)
{
    static int16_t Smooth_L_I = 0;
    static u8 Smooth_L_Val = 1;

    if (GPIO_ReadInputDataBit(GPIO_GRASP_1, Smooth_L_PIN) == 1)
    {
        Smooth_L_I++;
    }
    else if (GPIO_ReadInputDataBit(GPIO_GRASP_1, Smooth_L_PIN) == 0)
    {
        Smooth_L_I--;
    }

    if (Smooth_L_I >= FilteNum)
    {
        Smooth_L_Val = 1;
        Smooth_L_I = 0;
    }
    else if (Smooth_L_I <= -FilteNum)
    {
        Smooth_L_Val = 0;
        Smooth_L_I = 0;
    }

    return Smooth_L_Val;
}

/*ƽ���Ҵ�����*/
u8 Return_Smooth_R_Val(void)
{
    static int16_t Smooth_R_I = 0;
    static u8 Smooth_R_Val = 1;

    if (GPIO_ReadInputDataBit(GPIO_GRASP_1, Smooth_R_PIN) == 1)
    {
        Smooth_R_I++;
    }
    else if (GPIO_ReadInputDataBit(GPIO_GRASP_1, Smooth_R_PIN) == 0)
    {
        Smooth_R_I--;
    }

    if (Smooth_R_I >= FilteNum)
    {
        Smooth_R_Val = 1;
        Smooth_R_I = 0;
    }
    else if (Smooth_R_I <= -FilteNum)
    {
        Smooth_R_Val = 0;
        Smooth_R_I = 0;
    }

    return Smooth_R_Val;
}

/*����ǰ������*/
/*static  u8 Return_Stretch_Val(void)
{
    static int16_t Stretch_F_I = 0;
    static u8 Stretch_F_Val = 1;

    if(GPIO_ReadInputDataBit(GPIO_GRASP_1, Stretch_PIN) == 1)
    {
        Stretch_F_I ++;
    }
    else if(GPIO_ReadInputDataBit(GPIO_GRASP_1, Stretch_PIN) == 0)
    {
        Stretch_F_I --;
    }

    if(Stretch_F_I >= FilteNum)
    {
        Stretch_F_Val = 1;
        Stretch_F_I = 0;
    }
    else if(Stretch_F_I <= -FilteNum)
    {
        Stretch_F_Val = 0;
        Stretch_F_I = 0;
    }

    return Stretch_F_Val;
}
 */

/*��ȡ��������ֵ*/
void Get_Sensor_Val(void)
{
    Sensor.Smooth_L = Return_Smooth_L_Val();
    Sensor.Smooth_R = Return_Smooth_R_Val();
    // Sensor.Stretch_F 	= Return_Stretch_Val();
}
