#ifndef __VISION_ISR_H_
#define __VISION_ISR_H_
#include "stm32f4xx.h"

typedef __packed struct
{
	u8 FLAG;
	u16 Vision_Len;
	float AUTO_PITCH;
	float AUTO_YAW;
	float PITCH_ERR;
	float YAW_ERR;
	float LAST_PITCH;
	float LAST_YAW;
	float DISTANCE;
}VISION_t;


/*视觉数据结构体初始化*/
void VISION_INIT(VISION_t*vision);



/*选择视觉的模式*/
u8 VISION_MODE_SELECT(u8 mode, u8 data);



/*返回视觉数据结构体的指针*/
VISION_t *Get_VISION_Point(void);

#endif
