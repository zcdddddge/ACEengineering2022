#ifndef __BOARDCOMMUNI_H_
#define __BOARDCOMMUNI_H_
#include "stm32f4xx.h"
#include "REMOTE_ISR.h"




/*���ͨ�ų�ʼ��*/
void BoardCommuni_Init(void);

/*����ң������*/
void Send_RC_To_Board(void);

/*���Ϳ���ָ��������*/
void Send_Ctrl_To_Board(unsigned char Grasp_Up, unsigned char Translation, unsigned char Telescoping, unsigned char Clap, unsigned char Flip);
/*���̽��ռ�ȡ����ָ��*/
void  BoardCommuni_DataUpdate(int16_t *speed) ;
#endif
