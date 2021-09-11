/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      开始任务。
  ******************************************************************************
  */

#ifndef __INIT_H__
#define __INIT_H__
#include "maths.h"
#include "filter.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

#define CHASSIS_APP 0
#define GIMBAL_APP  1



void task_init(void);
void sys_task(void);
void start_task(void);
void services_task(void const *argument);
void system_config(void);
void hw_init(void);
uint8_t get_sys_cfg(void);

void out_of_control(void);
void normal_control(void);

void calibrate_task_hang(void);
void calibrate_task_remove_hang(void);

#endif
