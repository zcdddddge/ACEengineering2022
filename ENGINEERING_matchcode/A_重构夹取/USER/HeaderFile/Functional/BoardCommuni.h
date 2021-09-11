#ifndef __BOARDCOMMUNI_H_
#define __BOARDCOMMUNI_H_
#include "stm32f4xx.h"
#include "REMOTE_ISR.h"
#include "GPIO_DEAL.h"

#define AUTO_GRASP_S1     3
#define AUTO_GRASP_S2     3 


typedef __packed struct 
{

	/* -----------------------------����Ϊ�½ṹ��------------------------ */
	unsigned char Grasp_Up : 2;	  //�Զ���ȡ
	unsigned char Translation  : 1; //��װ��λ
	unsigned char Telescoping : 1;	  //�ɼ�
	unsigned char Clap : 1;	  //�һ�һ��
	unsigned char Flip : 1;	  //�һ�����
}KeyBoard_State_t;

typedef __packed struct
{
	RC_ctrl_t Can_RC;
	u8 *Rc;
	KeyBoard_State_t state;
}Board_Communi_t;
	
void Board_Communi_Init(void);
void Board_Communi_Updata(void);
Board_Communi_t* Return_Board_Communi(void);
void Send_Crtl_To_Board(void);

#endif
