#include "visual.h"
#include "usart3.h"
#include "filter.h"
#include "fifo_buff.h"
#include "parameter.h"

Vision_Auto_Data_t vision_auto_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};
extern fifo_rx_def fifo_usart_rx_3;
fifo_rx_def *pfifo_visual = &fifo_usart_rx_3;


void automatic_aiming_init(void)
{
    usart3_init(3); //串口三初始化
}

//返回视觉自瞄控制变量，通过指针传递方式传递信息
Vision_Auto_Data_t *Get_Auto_Control_Point(void)
{
    return &vision_auto_data;
}

/*
*功能：发送数据到MiniPC（串口三）
*输入：8位数据自定义，8位数据自定义
*输出：无
*协议：帧头0xF5  校验段0xF5 0x00 0x00  帧尾：0xF6
*/
void visual_send_data(uint8_t data, uint8_t mode, uint8_t shoot_speed)
{
	uint8_t SendBuff[5];

    SendBuff[0] = 0xFF;
    SendBuff[1] = data;
    SendBuff[2] = mode;
    SendBuff[3] = shoot_speed;
    SendBuff[4] = 0xFE;
	
	usart3_dma_send(SendBuff, ARR_SIZE(SendBuff));
}

void visual_data_reception(void)
{	
	uint8_t buff_read[32];
	
	uint32_t length = fifo_read_buff(pfifo_visual, buff_read, ARR_SIZE(buff_read));
    if (length)
    {
		for (uint16_t i = 0; i < length; i++)
		{
			if (buff_read[i] == 0xFF && buff_read[i + 7] == 0xFE)
			{
				vision_auto_data.auto_yaw_angle = (hex2Float(buff_read[i + 2], buff_read[i + 1]) / 100);   //自动打击的y轴角度计算/100
				vision_auto_data.auto_pitch_angle = (hex2Float(buff_read[i + 4], buff_read[i + 3]) / 100); //自动打击的p轴角度计算
				vision_auto_data.auto_kalman_yaw_angle = (hex2Float(buff_read[i + 6], buff_read[i + 5]) / 100);   //自动打击的y轴角度计算/100

				//vision_auto_data.len = (hex2Float(MSDataBuffer[i + 6], MSDataBuffer[i + 5]) / 100);              //距离

				i = i + 7;
			}
		}		
	}
	else
	{
		//没有数据
	}
	
	if (pfifo_visual->error)
    {
		// 接收错误
        pfifo_visual->error = 0; 
    }
}

/*
*功能：视觉卡尔曼数据初始化
*
*/
void MiniPC_Kalman_Data_Init(void)
{
    {
        kalman_Filter_Init.A_data[0] = 1;
        kalman_Filter_Init.A_data[2] = 1;
        kalman_Filter_Init.A_data[5] = 1;
        kalman_Filter_Init.A_data[7] = 1;
        kalman_Filter_Init.A_data[10] = 1;
        kalman_Filter_Init.A_data[15] = 1;
        //观测矩阵
        kalman_Filter_Init.H_data[0] = 1;
        kalman_Filter_Init.H_data[5] = 1;
        //状态转移协方差矩阵
        kalman_Filter_Init.P_data[0] = 1;
        kalman_Filter_Init.P_data[5] = 1;
        kalman_Filter_Init.P_data[10] = 1;
        kalman_Filter_Init.P_data[15] = 1;
        //观测噪声方差
        kalman_Filter_Init.R_data[0] = 1;
        kalman_Filter_Init.R_data[3] = 1;
        //状态转移协方差矩阵
        kalman_Filter_Init.Q_data[0] = 10;
        kalman_Filter_Init.Q_data[5] = 10;
        kalman_Filter_Init.Q_data[10] = 0.01;
        kalman_Filter_Init.Q_data[15] = 0.01;
    }
    Kalman_Filter_Init(&kalman_Filter, &kalman_Filter_Init);
}

/*
*功能：高低八位数据整合
*/
float hex2Float(uint8_t HighByte, uint8_t LowByte)
{
    float high = (float)(HighByte & 0x7f);
    float low = (float)LowByte;

    if (HighByte & 0x80) //MSB is 1 means a negative number
    {
        return (high * 256.0f + low) - 32768;
    }
    else
    {
        return (high * 256.0f + low);
    }
}
