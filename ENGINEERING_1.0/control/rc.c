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
ch0为右边的左右
ch1为右边的上下
ch2为左边的左右
ch3为左边的上下
ch4为拨弹（左上角）
s1为左上
s2为右上
*/

/*****************************操作键位说明书**********************************************************/
/* 一、遥控模式：
         1.底盘跟随  ：左中右上
         2.扭腰模式  ：左下右上
         3.底盘小陀螺：左上右上
         4.打符模式  ：左上右中（底盘以云台坐标轴移动，不跟随）
         5.自瞄模式  ：左下右中
         6.键盘模式  ：左中右中
         7.发弹      ：在初始化完成且右开关打上或中情况下，左上角波轮拉最上发弹，回中停止射弹
         8.关机      ：右下
   二、键鼠模式：
         1.基本运动：WASD
         2.云台运动：鼠标
         3.发射：    鼠标左键单点单发，按住连发
         4.加速（电容放电）：    按住shift（逮虾户）
         5.扭腰模式： F  （按一下进入，再按一下返回）
         6.自瞄模式： 鼠标右键  （按住）
         7.补给模式：  G  （按一下云台往一边转，补完弹再按一下回来）
         8.高射速模式：C （按一次进入）（这个模式没用了，之前全部最大射速打）
         9.低射速模式：V （按一次进入）
         10.退弹模式： Z  （按住）
         11.炮台模式： Ctrl （按住，只能控制云台，底盘不动）
         12.打符模式： X （一次进入）
         13.小陀螺模式：R（按一次进入）
                                                                                                    */
/****************************************************************************************************/

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700
/* 超过这个时间没有收到新的遥控器数据就认为已经失控 */
#define REMOTE_LOST_TIME ((uint32_t)50) //50ms

rc_ctrl_t rc_ctrl;
int RC_FLAG = 0;
static portTickType RemoteLostTime = 0;

/**
  * @brief      遥控器数据死区限制
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
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  * @attention
  */
void remote_control_init(void)
{
    usart1_init(2, 1);
}

/**
  * @brief          返回遥控器控制变量，通过指针传递方式传递信息
  * @param[in]      none
  * @retval         返回遥控器控制变量 &rc_ctrl
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

/* 刷新失联时间 */
static void rc_lost_time_refresh(void)
{
    /* 当接收到数据时，刷新失联倒计时   xTaskGetTickCountFromISR  xTaskGetTickCount  */
    RemoteLostTime = xTaskGetTickCount() + REMOTE_LOST_TIME;
}

/**
  * @brief          判断遥控器数据是否出错
  * @param[in]      none
  * @retval         none
  * @attention
  */
uint8_t rc_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
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
    //	rc_ctrl.rc.s1 = RC_SW_ERROR;  //出现错误为0
    //	rc_ctrl.rc.s2 = RC_SW_ERROR;  //出现错误为0
    //	rc_ctrl.rc.s1 = RC_SW_MID;   //中
    //	rc_ctrl.rc.s2 = RC_SW_DOWN;  //下
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.kb.key_code = 0;

    //重启遥控器
    //    delay_ms(2);
    //    rc_restart(18);  //18
    //    delay_ms(2);

    return 1;
}

/**
  * @brief          摇杆量清零
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
    //	rc_ctrl.rc.s1 = RC_SW_ERROR;  //出现错误为0
    //	rc_ctrl.rc.s2 = RC_SW_ERROR;  //出现错误为0
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.kb.key_code = 0;
}

/**
  * @brief          遥控器重启
  * @param[in]      dma_buf_num：DMA 数据流传输的数据量大小
  * @retval         none
  * @attention
  */
void rc_restart(uint16_t dma_buf_num)
{
    USART_Cmd(USART1, DISABLE);
    DMA_Cmd(DMA2_Stream2, DISABLE);
    DMA_SetCurrDataCounter(DMA2_Stream2, dma_buf_num); //设置对应的 DMA 数据流传输的数据量大小

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

    /* 刷新失联倒计时 */
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
    rc_ctrl.rc.ch[4] = ((int16_t)sbus_buf[16] | ((int16_t)sbus_buf[17] << 8)) & 0x07FF; //遥控左上角拨轮

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET; // -通道中值1024
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET; // （ -660 ~ 660 ）
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;

    rc_ctrl.rc.ch[0] = rc_deadline_limit(rc_ctrl.rc.ch[0], 5); //死区限制
    rc_ctrl.rc.ch[1] = rc_deadline_limit(rc_ctrl.rc.ch[1], 5); //死区限制
    rc_ctrl.rc.ch[2] = rc_deadline_limit(rc_ctrl.rc.ch[2], 5); //死区限制
    rc_ctrl.rc.ch[3] = rc_deadline_limit(rc_ctrl.rc.ch[3], 5); //死区限制
}
