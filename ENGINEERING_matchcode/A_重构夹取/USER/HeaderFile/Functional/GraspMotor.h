#ifndef __GRASPMOTOR_H_
#define __GRASPMOTOR_H_
#include "motor.h"
#include "MotorAction.h"
#include "REMOTE_ISR.h"
#include "GPIO_DEAL.h"
#include "vL53L0.h"
#include "stm32f4xx.h"
#include "MathLib.h"
#include "FilterLib.h"

/****************************�������ṹ��*****************************************/
#define Update_Sensor_Val 			Get_Sensor_Val
#define Update_VL53L0_Val				VL53L0_Data_Deal

/****************************���͵����ֵ*****************************************/
#if 0
#define MotorOutput_201_204		Gr->GraspMotor[0].SPID.Out,Gr->GraspMotor[1].SPID.Out,Gr->GraspMotor[2].SPID.Out,Gr->GraspMotor[3].SPID.Out
#define MotorOutput_205_208		Gr->GraspMotor[4].SPID.Out,Gr->GraspMotor[5].SPID.Out,Gr->GraspMotor[6].SPID.Out,Gr->GraspMotor[7].SPID.Out
#endif 


#if 1
#define MotorOutput_201_204		0,0,0,0
#define MotorOutput_205_208		0,0,0,0
#endif 

static const int16_t Supply_Speed	      = 1000;


/*******************************���pid***********************************************/
/*��ת*/
#define GRASP_FLIP_S_P   	4.0f
#define GRASP_FLIP_S_I   	0.0f
#define GRASP_FLIP_S_D   	0.05f
#define GRASP_FLIP_P_P  	100.0f
#define	GRASP_FLIP_P_I   	0.0f
#define GRASP_FLIP_P_D   	0.0f
/*�н�*/
#define GRASP_CLIP_S_P   	3.5f
#define GRASP_CLIP_S_I   	0.0f
#define GRASP_CLIP_S_D   	0.02f
#define GRASP_CLIP_P_P  	40.7f
#define	GRASP_CLIP_P_I   	0.0f
#define GRASP_CLIP_P_D   	0.0f
/*����*/
#define GRASP_TRANS_S_P   	1.22f
#define GRASP_TRANS_S_I   	0.0f
#define GRASP_TRANS_S_D   	0.02f
#define GRASP_TRANS_P_P  		100.5f
#define	GRASP_TRANS_P_I   	0.0f
#define GRASP_TRANS_P_D   	0.0f
/*ƽ��*/
#define GRASP_NOROATE_S_P   	4.2f
#define GRASP_NOROATE_S_I   	0.0f
#define GRASP_NOROATE_S_D   	0.01f
#define GRASP_NOROATE_P_P  	120.0f
#define	GRASP_NOROATE_P_I   	0.0f
#define GRASP_NOROATE_P_D   	20.0f
/*̧��*/
#define GRASP_UPLIFT_S_P   	4.0f
#define GRASP_UPLIFT_S_I   	0.0f
#define GRASP_UPLIFT_S_D   	0.2f
#define GRASP_UPLIFT_P_P  	400.0f  // 350.5 
#define	GRASP_UPLIFT_P_I   	0.0f
#define GRASP_UPLIFT_P_D   	100.0f
/*���ӵ���*/
#define  GRASP_SUPPLY_S_P   1.22f
#define  GRASP_SUPPLY_S_I   0.0f
#define  GRASP_SUPPLY_S_D   0.02f
#define  GRASP_SUPPLY_P_P   100.5f
#define  GRASP_SUPPLY_P_I   0.0f
#define  GRASP_SUPPLY_P_D   0.1f

/*��ת*/
#define GRASP_ROTATE_S_P  	1.22f
#define	GRASP_ROTATE_S_I   	0.0f
#define GRASP_ROTATE_S_D   	0.2f
#define GRASP_ROTATE_P_P   	100.0f
#define GRASP_ROTATE_P_I   	0.0f
#define GRASP_ROTATE_P_D   	0.0f

/*��һ*/
#define GRASP_CENTER_S_P 1.22f
#define GRASP_CENTER_S_I 0.0f
#define GRASP_CENTER_S_D 0.2f
#define GRASP_CENTER_P_P 100.0f
#define GRASP_CENTER_P_I 0.0f
#define GRASP_CENTER_P_D 0.0f

/*��ȡ�ṹ��*/
typedef __packed struct
{
	Motor_t GraspMotor[8];
	Sensor_t *sensor;
	VL53L0_t *vL53L0;
	u8 state[4];
	u8 conversion_state;
	u8 reset[8];
	void (*Can_Send_Grasp_1)(int16_t,int16_t,int16_t,int16_t);
	void (*Can_Send_Grasp_2)(int16_t,int16_t,int16_t,int16_t);
	Encoder_t*(*Get_Encoder)(u8);
	Encoder_t*(*Get_Encoder2)(u8);
	Sensor_t* (*Get_Sensor_t)(void);
	VL53L0_t* (*Get_VL53L0_t)(void);
}Gr_t;



/*��ȡ�����ֳ�ʼ��*/
void Grasp_Motor_Init(Gr_t *Gr);

/*ң�ؿ���*/
void RC_Ctrl(Gr_t *Gr,RC_ctrl_t *rc);

/*�ϵ����*/
void Poweroff_Ctrl(Gr_t *Gr);

/*��λ��ȡ*/
void Reset_Grasp(Gr_t *Gr);

/*����*/
void bullet_Supply (Gr_t *Gr,Motor_t *Supply,int8_t  dire); 

/*���㺯����util���д��Ż�*/
void pid_Cala(Gr_t *Gr);


#endif
