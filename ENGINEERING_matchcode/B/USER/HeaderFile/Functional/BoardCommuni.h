#ifndef __BOARDCOMMUNI_H_
#define __BOARDCOMMUNI_H_
#include "stm32f4xx.h"
#include "REMOTE_ISR.h"




/*板间通信初始化*/
void BoardCommuni_Init(void);

/*发送遥控数据*/
void Send_RC_To_Board(void);

/*发送控制指令至板子*/
void Send_Ctrl_To_Board(unsigned char Grasp_Up, unsigned char Translation, unsigned char Telescoping, unsigned char Clap, unsigned char Flip);
/*底盘接收夹取控制指令*/
void  BoardCommuni_DataUpdate(int16_t *speed) ;
#endif
