#ifndef __MATHLIB_H_
#define __MATHLIB_H_
#include "stm32f4xx.h"

#define fp32 float


int16_t limit(int16_t x,int16_t max,int16_t min);
float float_limit(float x,float max,float min);
signed long limit_long(signed long x,signed long max,signed long min);
int16_t int16_t_abs(int16_t x);
signed long long_abs(signed long x);
float float_abs(float x);			
int16_t Dead_Zone(int16_t Value,int16_t Limit);
int16_t LIMIT_RAISE(int16_t raise,int16_t now,int16_t Limit);
int16_t RETURN_MAX(int16_t*Value,int16_t num);
float Angle_Limiting(float Angl_Err,float limit);


int16_t Remainder(int16_t Divisor,int16_t divisor);
int16_t Float_Sign(float NUM);
int16_t int16_t_Sign(int16_t NUM);


fp32 INOVERSHOOT(fp32 IN,fp32 NOW,fp32 REFER);

#endif
