#ifndef __REMOTEDEAL_H_
#define __REMOTEDEAL_H_
#include "REMOTE_ISR.h"
#include "FilterLib.h"



/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0) 														
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1) 														
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)
/* ----------------------- PC Key Definition-------------------------------- */


/*Remote结构体*/
typedef __packed struct
{
	RC_ctrl_t *RC_ctrl;
	First_Order_t RC_X;
	First_Order_t RC_Y;
	First_Order_t RC_Z;
	First_Order_t KM_X;
	First_Order_t KM_Y;
	First_Order_t KM_Z;
	RC_ctrl_t* (*Get_Remote_Point)(void);
}REMOTE_t;


/*Remote结构体初始化*/
void Remote_Data_Init(void);


/*Remote数据处理*/
void Remote_Data_Deal(void);


/*返回Remote数据指针*/
REMOTE_t *Return_RemoteDeal_Point(void);


#endif
