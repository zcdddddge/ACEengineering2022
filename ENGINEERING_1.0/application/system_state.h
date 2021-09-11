#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__
#include "struct_typedef.h" 


typedef enum
{
	INF_STOP = 0, //Í£Ö¹
	INF_RC,       //Ò£¿ØÆ÷
	INF_MK,       //¼üÅÌ
} infantry_state_e;


void system_state_set(infantry_state_e state);
infantry_state_e system_state_return(void);

#endif



