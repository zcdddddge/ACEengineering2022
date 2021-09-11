#ifndef __GRASP_FSM_H_
#define __GRASP_FSM_H_

#include "fsm.h" 

#define MAZAGINE_CLOSE    Grasp.Bullet_Supply(&Grasp.Gr, &Grasp.Gr.GraspMotor[6],-1)
#define MAZAGINE_OPEN     Grasp.Bullet_Supply(&Grasp.Gr, &Grasp.Gr.GraspMotor[6],1) 

#define GRASP_LIFT				uplift(&Grasp.Gr.GraspMotor[0], Grasp.Gr.vL53L0, 16.3f, 1);
#define CONVERSION_LIFT		uplift(&Grasp.Gr.GraspMotor[0], Grasp.Gr.vL53L0, 30.5f, 1);
#define DOWN							uplift(&Grasp.Gr.GraspMotor[0], Grasp.Gr.vL53L0, 1.0f, 2);

#define RALL_FORWARD			Translation(&Grasp.Gr.GraspMotor[1], 1);
#define RALL_BACK					Translation(&Grasp.Gr.GraspMotor[1], 2);

#define SLIDE_FORWARD			Telescoping(&Grasp.Gr.GraspMotor[2], 1);
#define SLIDE_BACK				Telescoping(&Grasp.Gr.GraspMotor[2], 2);

#define TURN_OVER					flip2(&Grasp.Gr.GraspMotor[4], 192, 10, 1);
#define TIP_BACK					flip2(&Grasp.Gr.GraspMotor[4], 0, 5, 2);

#define CLAMPING					clamp(&Grasp.Gr.GraspMotor[3], 270, 1);
#define PINE_CLIP					clamp(&Grasp.Gr.GraspMotor[3], 0, 2);

#define BULLY_SUPLY       0      //Ò£¿Ø¿ØÖÆµ¯²Ö









void Chassis_FSM_Init(void);

FSM_t *Return_Chassis_Fsm(void) ;

#endif 
