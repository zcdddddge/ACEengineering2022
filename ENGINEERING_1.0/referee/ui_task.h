#ifndef __USER_TASK_H
#define __USER_TASK_H
#include "struct_typedef.h"

extern void ui_task(void *pvParameters);
void ui_dynamic_armor_plate(void);
void ui_chassis_mode(int chassis_ui);
void ui_gimbal_mode(void);
void pitch_draw(void);
void projectile_bullet_draw(void);
void projectile_hit_draw(void);
uint16_t projectile_be_hit(void);
#endif
