#ifndef __ROBOTACTION_H_
#define __ROBOTACTION_H_

#include  "GraspMotor.h"
#include  "MotorAction.h" 
#include  "GPIO_DEAL.h"


void Auto_Ctrl(Gr_t *Gr,u8 box);		/*×Ô¶¯¿ØÖÆ*/
void auto_grasp(Gr_t *Gr);

void Reset(Gr_t *Gr);

#endif
