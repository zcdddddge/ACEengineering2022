#ifndef __DELAY_H__
#define __DELAY_H__
#include "struct_typedef.h"


//0,��֧��os
//1,֧��os
#define SYSTEM_SUPPORT_OS		1		//����ϵͳ�ļ����Ƿ�֧��OS

void delay_init(u8 SYSCLK);
void delay_us(u32 nus);
void delay_ms(u32 nms);
void delay_xms(u32 nms);

//void delay_ms(u16 nms);
//void delay_us(u32 nus);

#endif





























