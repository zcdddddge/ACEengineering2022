#ifndef __GPIO_DEAL_H_
#define __GPIO_DEAL_H_
#include "stm32f4xx.h"


/*返回YAW归中传感器的检测值*/
u8 Return_YAW_Calibrate_Val(void);



/*摄像头切换开关，choice为：
	1 - 开启夹取摄像头	2 - 开启救援摄像头	3 - 关闭所有摄像头*/
void CAMERA_SWITCH(u8 choice);


#endif
