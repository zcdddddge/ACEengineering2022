#ifndef __GIMBAL_BEHAVIOUR_H
#define __GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"
#include "gimbal_task.h"
#include "shoot_task.h"

#define  UpLift_Speed 			 -3000
#define  Telescoping_Speed   10000
#define  Cilp_Speed 	 			 -7000





typedef enum
{
	GIMBAL_ZERO_FORCE = 0, //云台无力
	GIMBAL_MANUAL,         //手动状态
	GIMBAL_AUTOATTACK,     //自瞄状态
	GIMBAL_AUTOBUFF,       //打符状态
	GIMBAL_REPLENISHMEN,   //补给状态
	GIMBAL_DOUBLE_GIMBAL,  //双云台状态（主云台自瞄，副云台操作）


	GRASP_INIT,
	GRASP_RESET,
	GRASP_FIRST,
	GRASP_SECOND,
	GRASP_THIRD,
	FLIP_RESET,
} gimbal_behaviour_e;

extern void gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f);
extern void gimbal_ch_retain(gimbal_control_t *gimbal_ch_f);
extern const gimbal_behaviour_e *get_gimbal_behaviour_point(void);
extern void gimbal_ch_set0(void);
#endif
