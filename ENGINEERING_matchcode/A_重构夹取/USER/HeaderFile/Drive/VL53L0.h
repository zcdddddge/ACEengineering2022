#ifndef __VL53L0_H_
#define __VL53L0_H_
#include "stm32f4xx.h"

typedef __packed struct
{
	u8 	data[64];
	float InitDistance;
	float distance; 
	u8 	receive_flag;
}VL53L0_t;


/*����VL53L0���ṹ��*/
VL53L0_t* Return_VL53L0_t(void);

/*����VL53L0�������*/
void VL53L0_Data_Deal(void);
	
#endif
