#include "rc.h"
#include "iwdg.h"
#include "delay.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "maths.h"
#include "usart1.h"
#include "parameter.h"

/*
ch0Ϊ�ұߵ�����
ch1Ϊ�ұߵ�����
ch2Ϊ��ߵ�����
ch3Ϊ��ߵ�����
ch4Ϊ���������Ͻǣ�
s1Ϊ����
s2Ϊ����
*/

/*****************************������λ˵����**********************************************************/
/* һ��ң��ģʽ��
         1.���̸���  ����������
         2.Ť��ģʽ  ����������
         3.����С���ݣ���������
         4.���ģʽ  ���������У���������̨�������ƶ��������棩
         5.����ģʽ  ����������
         6.����ģʽ  ����������
         7.����      ���ڳ�ʼ��������ҿ��ش��ϻ�������£����Ͻǲ��������Ϸ���������ֹͣ�䵯
         8.�ػ�      ������
   ��������ģʽ��
         1.�����˶���WASD
         2.��̨�˶������
         3.���䣺    ���������㵥������ס����
         4.���٣����ݷŵ磩��    ��סshift����Ϻ����
         5.Ť��ģʽ�� F  ����һ�½��룬�ٰ�һ�·��أ�
         6.����ģʽ�� ����Ҽ�  ����ס��
         7.����ģʽ��  G  ����һ����̨��һ��ת�����굯�ٰ�һ�»�����
         8.������ģʽ��C ����һ�ν��룩�����ģʽû���ˣ�֮ǰȫ��������ٴ�
         9.������ģʽ��V ����һ�ν��룩
         10.�˵�ģʽ�� Z  ����ס��
         11.��̨ģʽ�� Ctrl ����ס��ֻ�ܿ�����̨�����̲�����
         12.���ģʽ�� X ��һ�ν��룩
         13.С����ģʽ��R����һ�ν��룩
                                                                                                    */
/****************************************************************************************************/

//ң����������������
#define RC_CHANNAL_ERROR_VALUE 700
/* �������ʱ��û���յ��µ�ң�������ݾ���Ϊ�Ѿ�ʧ�� */
#define REMOTE_LOST_TIME ((uint32_t)50) //50ms

rc_ctrl_t rc_ctrl;
int RC_FLAG = 0;
static portTickType RemoteLostTime = 0;

/**
  * @brief      ң����������������
  * @param[in]  input
  * @param[in]  dealine
  * @retval     
*/
int16_t rc_deadline_limit(int16_t input, int16_t dealine)
{
    if (input > dealine || input < -dealine)
    {
        return input;
    }
    else
    {
        return 0;
    }
}

static void rc_lost_time_refresh(void);

/**
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  * @attention
  */
void remote_control_init(void)
{
    usart1_init(2, 1);
}

/**
  * @brief          ����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
  * @param[in]      none
  * @retval         ����ң�������Ʊ��� &rc_ctrl
  * @attention
  */
const rc_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

uint32_t chassis_rc_lost_time(void)
{
    return RemoteLostTime;
}

/* ˢ��ʧ��ʱ�� */
static void rc_lost_time_refresh(void)
{
    /* �����յ�����ʱ��ˢ��ʧ������ʱ   xTaskGetTickCountFromISR  xTaskGetTickCount  */
    RemoteLostTime = xTaskGetTickCount() + REMOTE_LOST_TIME;
}

/**
  * @brief          �ж�ң���������Ƿ����
  * @param[in]      none
  * @retval         none
  * @attention
  */
uint8_t rc_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    do
    {
        if (int16_t_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
        {
            break;
        }

        if (int16_t_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
        {
            break;
        }

        if (int16_t_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
        {
            break;
        }

        if (int16_t_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
        {
            break;
        }

        if (rc_ctrl.rc.s1 == RC_SW_ERROR)
        {
            break;
        }

        if (rc_ctrl.rc.s2 == RC_SW_ERROR)
        {
            break;
        }
        return 0;
    } while (0);

    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    //	rc_ctrl.rc.s1 = RC_SW_ERROR;  //���ִ���Ϊ0
    //	rc_ctrl.rc.s2 = RC_SW_ERROR;  //���ִ���Ϊ0
    //	rc_ctrl.rc.s1 = RC_SW_MID;   //��
    //	rc_ctrl.rc.s2 = RC_SW_DOWN;  //��
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.kb.key_code = 0;

    //����ң����
    //    delay_ms(2);
    //    rc_restart(18);  //18
    //    delay_ms(2);

    return 1;
}

/**
  * @brief          ҡ��������
  * @param[in]      none
  * @retval         none
  * @attention
  */
void remote_reload(void)
{
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    //	rc_ctrl.rc.s1 = RC_SW_ERROR;  //���ִ���Ϊ0
    //	rc_ctrl.rc.s2 = RC_SW_ERROR;  //���ִ���Ϊ0
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.kb.key_code = 0;
}

/**
  * @brief          ң��������
  * @param[in]      dma_buf_num��DMA �������������������С
  * @retval         none
  * @attention
  */
void rc_restart(uint16_t dma_buf_num)
{
    USART_Cmd(USART1, DISABLE);
    DMA_Cmd(DMA2_Stream2, DISABLE);
    DMA_SetCurrDataCounter(DMA2_Stream2, dma_buf_num); //���ö�Ӧ�� DMA �������������������С

    USART_ClearFlag(USART1, USART_FLAG_IDLE);

    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
    DMA_Cmd(DMA2_Stream2, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void rc_shut_down(void)
{
    USART_Cmd(USART1, DISABLE);
    DMA_Cmd(DMA2_Stream2, DISABLE);
}

void chassis_rc_callback(volatile const uint8_t *sbus_buf)
{
    if (sbus_buf == NULL)
        return;

#ifdef WATCH_DOG
    {
        RC_FLAG++;
        if (5 == RC_FLAG)
        {
            iwdg_feed();
            RC_FLAG = 0;
        }
    }
#endif

    /* ˢ��ʧ������ʱ */
    rc_lost_time_refresh();

    rc_ctrl.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                        (sbus_buf[4] << 10)) &
                       0x07ff;
    rc_ctrl.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;              //!< Channel 3
    rc_ctrl.rc.s1 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                                 //!< Switch left
    rc_ctrl.rc.s2 = ((sbus_buf[5] >> 4) & 0x0003);                                      //!< Switch right
    rc_ctrl.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                                 //!< Mouse X axis
    rc_ctrl.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                                 //!< Mouse Y axis
    rc_ctrl.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                               //!< Mouse Z axis
    rc_ctrl.mouse.press_l = sbus_buf[12];                                               //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = sbus_buf[13];                                               //!< Mouse Right Is Press ?
    rc_ctrl.kb.key_code = sbus_buf[14] | (sbus_buf[15] << 8);                           //!< KeyBoard value
    rc_ctrl.rc.ch[4] = ((int16_t)sbus_buf[16] | ((int16_t)sbus_buf[17] << 8)) & 0x07FF; //ң�����Ͻǲ���

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET; // -ͨ����ֵ1024
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET; // �� -660 ~ 660 ��
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;

    rc_ctrl.rc.ch[0] = rc_deadline_limit(rc_ctrl.rc.ch[0], 5); //��������
    rc_ctrl.rc.ch[1] = rc_deadline_limit(rc_ctrl.rc.ch[1], 5); //��������
    rc_ctrl.rc.ch[2] = rc_deadline_limit(rc_ctrl.rc.ch[2], 5); //��������
    rc_ctrl.rc.ch[3] = rc_deadline_limit(rc_ctrl.rc.ch[3], 5); //��������
}
