#ifndef __PWM_H__
#define __PWM_H__
#include "struct_typedef.h"

#define REVERSE_LIGHT_COUPLING  (1) //等于1，使用反向光耦


//#define Gimbal_Pitch_Output  TIM3->CCR3  //PB0


extern void friction_wheel_init(void);
extern void steering_gear_init(void);

extern void tim12_pwm_init(u32 arr, u32 psc);
extern void tim5_pwm_init(u32 arr, u32 psc);
extern void tim3_pwm_init(u32 arr, u32 psc);
extern void tim3_pwm_6020_init(u32 arr, u32 psc);
extern void tim4_pwm_init(u32 arr, u32 psc);



#endif
