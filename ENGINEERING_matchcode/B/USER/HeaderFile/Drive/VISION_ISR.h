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


/*�Ӿ����ݽṹ���ʼ��*/
void VISION_INIT(VISION_t*vision);



/*ѡ���Ӿ���ģʽ*/
u8 VISION_MODE_SELECT(u8 mode, u8 data);



/*�����Ӿ����ݽṹ���ָ��*/
VISION_t *Get_VISION_Point(void);

#endif
