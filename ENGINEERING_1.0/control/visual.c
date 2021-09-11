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
    usart3_init(3); //��������ʼ��
}

//�����Ӿ�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
Vision_Auto_Data_t *Get_Auto_Control_Point(void)
{
    return &vision_auto_data;
}

/*
*���ܣ��������ݵ�MiniPC����������
*���룺8λ�����Զ��壬8λ�����Զ���
*�������
*Э�飺֡ͷ0xF5  У���0xF5 0x00 0x00  ֡β��0xF6
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
				vision_auto_data.auto_yaw_angle = (hex2Float(buff_read[i + 2], buff_read[i + 1]) / 100);   //�Զ������y��Ƕȼ���/100
				vision_auto_data.auto_pitch_angle = (hex2Float(buff_read[i + 4], buff_read[i + 3]) / 100); //�Զ������p��Ƕȼ���
				vision_auto_data.auto_kalman_yaw_angle = (hex2Float(buff_read[i + 6], buff_read[i + 5]) / 100);   //�Զ������y��Ƕȼ���/100

				//vision_auto_data.len = (hex2Float(MSDataBuffer[i + 6], MSDataBuffer[i + 5]) / 100);              //����

				i = i + 7;
			}
		}		
	}
	else
	{
		//û������
	}
	
	if (pfifo_visual->error)
    {
		// ���մ���
        pfifo_visual->error = 0; 
    }
}

/*
*���ܣ��Ӿ����������ݳ�ʼ��
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
        //�۲����
        kalman_Filter_Init.H_data[0] = 1;
        kalman_Filter_Init.H_data[5] = 1;
        //״̬ת��Э�������
        kalman_Filter_Init.P_data[0] = 1;
        kalman_Filter_Init.P_data[5] = 1;
        kalman_Filter_Init.P_data[10] = 1;
        kalman_Filter_Init.P_data[15] = 1;
        //�۲���������
        kalman_Filter_Init.R_data[0] = 1;
        kalman_Filter_Init.R_data[3] = 1;
        //״̬ת��Э�������
        kalman_Filter_Init.Q_data[0] = 10;
        kalman_Filter_Init.Q_data[5] = 10;
        kalman_Filter_Init.Q_data[10] = 0.01;
        kalman_Filter_Init.Q_data[15] = 0.01;
    }
    Kalman_Filter_Init(&kalman_Filter, &kalman_Filter_Init);
}

/*
*���ܣ��ߵͰ�λ��������
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
