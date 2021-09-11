#ifndef __CHASSIS_BEHAVIOUR_H__
#define __CHASSIS_BEHAVIOUR_H__
#include "struct_typedef.h"
#include "chassis_task.h"

#define Rescue_Speed     2400;   //��Ԯ����ٶ�

typedef enum
{
	CHASSIS_ZERO_FORCE = 0, //��������
	
	CHASSIS_FOLLOW,     //����
    CHASSIS_NO_FOLLOW,  //������
    CHASSIS_TWIST_WAIST, //Ť��
    CHASSIS_ROTATION,    //С����
	CHASSIS_BATTERY,     //��̨ģʽ
	

	
	
} chassis_behaviour_e;

typedef enum
{
	RESCUE_BACK,
	RESCUE_FORWARD,
	
	RESCLAP_BACK,
	RESCLAP_FORWARD,
	
} rescuemotor_behaviour_e;






extern void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f);
extern void Chassis_Stop(chassis_control_t *Chassis_Stop_f);
extern const chassis_behaviour_e *get_chassis_behaviour_point(void);
extern uint16_t re_chassis_behaviour(void);

#endif



