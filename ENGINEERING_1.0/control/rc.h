#ifndef __RC_H__
#define __RC_H__
#include "struct_typedef.h"


//������ջ���
#define SBUS_RX_BUF_NUM   18


/* ң���ĵ�����-BEGIN */
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN            ((uint16_t)364 )			//ͨ����Сֵ
#define RC_CH_VALUE_OFFSET         ((uint16_t)1024)			//ͨ���м�ֵ
#define RC_CH_VALUE_MAX            ((uint16_t)1684)			//ͨ�����ֵ
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_ERROR                ((uint16_t)0) //�������ش���
#define RC_SW_UP                   ((uint16_t)1)
#define RC_SW_MID                  ((uint16_t)3)
#define RC_SW_DOWN                 ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W       ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S       ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A       ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D       ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT   ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL    ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q       ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E       ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R       ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F       ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G       ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z       ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X       ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C       ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V       ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B       ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */


typedef struct //ң��ͳһ�ṹ��
{
	struct
	{
		int16_t ch[5];
		char s1;
		char s2;
	} rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
		
		
	} mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } kb;
//	__packed struct
//	{
//		uint16_t v;
//	} key;

} rc_ctrl_t;


//ң������ʼ��
extern void remote_control_init(void);
//��ȡң����ָ��
extern const rc_ctrl_t *get_remote_control_point(void);
//ң�����ݴ���
extern void rc_deal(rc_ctrl_t *rc_ctrl, volatile const uint8_t *sbus_buf);
//�ж�ң���������Ƿ��쳣
extern u8 rc_data_is_error(void);
//ң��������
extern void rc_restart(uint16_t dma_buf_num);
//ң������ֵ����
extern void remote_reload(void);

extern char return_rc_s1num(void);
extern char return_rc_s2num(void);

void rc_shut_down(void);
uint32_t chassis_rc_lost_time(void);

extern void chassis_rc_callback(volatile const uint8_t *sbus_buf);

#endif









