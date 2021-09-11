#ifndef __REMOTE_ISR_H_
#define __REMOTE_ISR_H_
#include "stm32f4xx.h"


/**************??&??*************************************/
typedef __packed struct
{
	int16_t x;              //????
	int16_t y;				     //????
	int16_t z; 
	int16_t kv0;
	int16_t kv1;
	int16_t kv2;
	int16_t kv3;        
	uint8_t press_l;       //????
	uint8_t press_r;       //????
	uint16_t key;
}Key_Mouse_t;

/************RC??????******************************/
typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t sw;
	int8_t s1;
	int8_t s2;
	int8_t Flag;
	Key_Mouse_t KV;
}RC_ctrl_t;


/*返回遥控器数据指针*/
RC_ctrl_t *Return_Remote_Point(void);


#endif
