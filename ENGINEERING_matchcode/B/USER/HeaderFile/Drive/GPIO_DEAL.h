#ifndef __GPIO_DEAL_H_
#define __GPIO_DEAL_H_
#include "stm32f4xx.h"


/*����YAW���д������ļ��ֵ*/
u8 Return_YAW_Calibrate_Val(void);



/*����ͷ�л����أ�choiceΪ��
	1 - ������ȡ����ͷ	2 - ������Ԯ����ͷ	3 - �ر���������ͷ*/
void CAMERA_SWITCH(u8 choice);


#endif
