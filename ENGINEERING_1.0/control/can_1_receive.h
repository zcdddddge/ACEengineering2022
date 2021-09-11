/**
  ******************************************************************************
  * @file       CAN_1_Receive.c/h
  * @brief      CAN1���պͷ��͵������
  ******************************************************************************
  */

#ifndef __CAN_1_RECEIVE_H__
#define __CAN_1_RECEIVE_H__
#include "struct_typedef.h"


//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t position;
    int16_t  speed;
    int16_t  given_current;
    uint8_t  temperate;
    int16_t  last_position;
	
	int16_t speed_filt;
	int16_t first_Flag;
	int32_t actual_Position;
} motor_measure_t;

typedef struct
{		
	float input_voltage;  //�����ѹ
	float Capacitance_voltage;  //���ݵ�ѹ
	float Input_current;  //�������
	float Set_power;  //�趨����
} Supercapacitor_receive_t;


const Supercapacitor_receive_t *get_supercap_control_point(void);
const motor_measure_t *get_fire_motor_measure_point(void);
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
const motor_measure_t *get_spitch_motor_measure_point(void);
const motor_measure_t *get_syaw_motor_measure_point(void);
const motor_measure_t *get_rescue_motor_measure_point(void);
const motor_measure_t *get_rescue_clap_motor_measure_point(void);

const motor_measure_t *get_uplift_measure_point(void);
const motor_measure_t *get_translation_measure_point(void);
const motor_measure_t *get_telescoping_measure_point(void);
const motor_measure_t *get_clip_measure_point(void);
const motor_measure_t *get_flip_measure_point(void);



//�������ݵ�ѹ
float re_capacitance_voltage(void);

extern void (*can1_callback)(CanRxMsg *);
void chassis_can1_callback(CanRxMsg *rx_message);
void gimbal_can1_callback(CanRxMsg *rx_message);

#endif
