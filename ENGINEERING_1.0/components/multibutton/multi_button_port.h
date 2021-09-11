#ifndef __MULTI_BUTTON_PORT_H_
#define __MULTI_BUTTON_PORT_H_
#include "stdint.h"
#include "string.h"
#include "multi_button.h"
#include "task_chassis.h"

typedef struct
{
    const rc_ctrl_t *button_rc; //Ò£¿ØÆ÷Ö¸Õë
	chassis_control_t *chassis_button; //Ò£¿ØÆ÷Ö¸Õë
	
	//ÉêÇë°´¼ü½á¹¹
	Button key_w;
	Button key_s;
	Button key_a;
	Button key_d;
	Button key_shift;
	Button key_ctrl;
	Button key_q;
	Button key_e;
	Button key_r;
	Button key_f;
	Button key_g;
	Button key_z;
	Button key_x;
	Button key_c;
	Button key_v;
	Button restart;
	Button mouse_r;
	Button mouse_l;
	//Button restart;
} button_control_t;

void button_all_init(void);
#endif
