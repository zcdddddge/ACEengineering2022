/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       upper_machine.c/h
  * @brief      ��λ��ͨѶ
  * @note       
  * @history    2021.07.08
  *
  @verbatim   
  ==============================================================================
  ���� VOFA+ ��λ�����е��Ժ�pid��������
    - �ֽڽ������빴ѡʮ�����ƣ���ʮ�����Ʒ�ʽ��ӡ�ַ�������ֻ�ܴ�ӡ���롣
  ʹ�� VOFA+ �е� JustFloat Э�飺
    - Э���ص�
  ��Э����С�˸���������ʽ���ֽ���Э�飬��ʮ�����Ƹ��㴫�䣬��ʡ����
  ��Э��ǳ��ʺ�����ͨ�������ࡢ����Ƶ�ʸߵ�ʱ��  

  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  */
#include "upper_machine.h"
#include "usart2.h"
#include "fifo_buff.h"
#include <string.h>
#include <stdlib.h>
#include "maths.h"
#include "parameter.h"

extern fifo_rx_def fifo_usart_rx_2;
static fifo_rx_def *pfifo_upper_computer = &fifo_usart_rx_2;

void upper_machine_communication(void)
{
    usart2_init(4);
}


/* pid�������� */
void pid_parameter_receive(pid_parameter_t *pid_speed, pid_parameter_t *pid_position)
{
    uint16_t i = 0;
    uint8_t buff_read[32];
    char buff[16];
    char *pid_p;

    uint32_t length = fifo_read_buff(pfifo_upper_computer, buff_read, ARR_SIZE(buff_read));
    if (length)
    {
        if ((uint8_t *)strstr((char *)buff_read, (char *)"nep p_Kp:"))
        {
            pid_p = strstr((char *)buff_read, (char *)"nep p_Kp:");
            for (i = 0; i < (length - ((int)buff_read - (int)pid_p)); i++)
            {
                if (*(pid_p + i) == 0x0A)
                {
                    strncpy(buff, pid_p + 9, i);
                    pid_position->Kp = atof(buff);
					memset(buff, 0, 16);
                }
            }
        }
        else if ((uint8_t *)strstr((char *)buff_read, (char *)"nep p_Ki:"))
        {
            pid_p = strstr((char *)buff_read, (char *)"nep p_Ki:");
            for (i = 0; i < (length - ((int)buff_read - (int)pid_p)); i++)
            {
                if (*(pid_p + i) == 0x0A)
                {
                    strncpy(buff, pid_p + 9, i);
                    pid_position->Ki = atof(buff);
					memset(buff, 0, 16);
                }
            }
        }
        else if ((uint8_t *)strstr((char *)buff_read, (char *)"nep p_Kd:"))
        {
            pid_p = strstr((char *)buff_read, (char *)"nep p_Kd:");
            for (i = 0; i < (length - ((int)buff_read - (int)pid_p)); i++)
            {
                if (*(pid_p + i) == 0x0A)
                {
                    strncpy(buff, pid_p + 9, i);
                    pid_position->Kd = atof(buff);
					memset(buff, 0, 16);
                }
            }
        }
        else if ((uint8_t *)strstr((char *)buff_read, (char *)"nep s_Kp:"))
        {
            pid_p = strstr((char *)buff_read, (char *)"nep s_Kp:");
            for (i = 0; i < (length - ((int)buff_read - (int)pid_p)); i++)
            {
                if (*(pid_p + i) == 0x0A)
                {
                    strncpy(buff, pid_p + 9, i);
                    pid_speed->Kp = atof(buff);
					memset(buff, 0, 16);
                }
            }
        }
        else if ((uint8_t *)strstr((char *)buff_read, (char *)"nep s_Ki:"))
        {
            pid_p = strstr((char *)buff_read, (char *)"nep s_Ki:");
            for (i = 0; i < (length - ((int)buff_read - (int)pid_p)); i++)
            {
                if (*(pid_p + i) == 0x0A)
                {
                    strncpy(buff, pid_p + 9, i);
                    pid_speed->Ki = atof(buff);
					memset(buff, 0, 16);
                }
            }
        }
        else if ((uint8_t *)strstr((char *)buff_read, (char *)"nep s_Kd:"))
        {
            pid_p = strstr((char *)buff_read, (char *)"nep s_Kd:");
            for (i = 0; i < (length - ((int)buff_read - (int)pid_p)); i++)
            {
                if (*(pid_p + i) == 0x0A)
                {
                    strncpy(buff, pid_p + 9, i);
                    pid_speed->Kd = atof(buff);
					memset(buff, 0, 16);
                }
            }
        }
    }
    else
    {
        // printf("no data rx");// û������
    }

    if (pfifo_upper_computer->error)
    {
		// ���մ���
        pfifo_upper_computer->error = 0; 
    }
}

uint8_t send_frame[16];
void package_frame(const void *data, int x)
{
    int i;
    for (i = 0; i < (4 * x); i++)
    {
        send_frame[i] = ((uint8_t *)data)[i];
    }
}
float vofa_t = 0.0f;
float data[4];
void vofa_test(void)
{

    int i;

    vofa_t += 0.1f;
    data[0] = sin_calculate(vofa_t);
    data[1] = sin_calculate(2.0f * vofa_t);
    data[2] = sin_calculate(3.0f * vofa_t);
    data[3] = sin_calculate(4.0f * vofa_t);

    package_frame(data, 4);

    //	for (i = 0; i < 1; i++)
    //	{
    //		send_frame[i] = (uint8_t)(data[i] >> 24);
    //		send_frame[i+1] = (uint8_t)(data[i] >> 16);
    //		send_frame[i+2] = (uint8_t)(data[i] >> 8);
    //		send_frame[i+3] = (uint8_t)(data[i]);
    //	}

    for (i = 0; i < 16; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
            ;                                           //�������ݵ�����2
        USART_SendData(USART2, (uint8_t)send_frame[i]); //�ȴ��ϴδ������
    }

    // ����֡β
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    for (int i = 0; i < 4; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
            ;                                     //�������ݵ�����2
        USART_SendData(USART2, (uint8_t)tail[i]); //�ȴ��ϴδ������
    }
}
