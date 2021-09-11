#include "RemoteDeal.h"
#include "MathLib.h"

/*ң�ؽṹ��*/
REMOTE_t REMOTE;
/*ң������*/
static const u8 DeadZone = 10;


/*************************************************************************************************
*����:	Get_RemoteDeal_Point
*����:	���ش�����ң����ֵ���Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
REMOTE_t *Return_RemoteDeal_Point(void)
{
    return &REMOTE;
}

/*************************************************************************************************
*����:	Remote_Data_Init
*����:	ң����ֵ�����ʼ��
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
void Remote_Data_Init(void)
{
    /*����ӳ��*/
    REMOTE.Get_Remote_Point = Return_Remote_Point;

    /*ң����ֵ��ʼ��*/
    REMOTE.RC_ctrl->ch0 = REMOTE.RC_ctrl->ch1 = REMOTE.RC_ctrl->ch2 = REMOTE.RC_ctrl->ch3 = 0;
    REMOTE.RC_ctrl->s1 = REMOTE.RC_ctrl->s2 = REMOTE.RC_ctrl->sw = REMOTE.RC_ctrl->Flag = 0;
    REMOTE.RC_ctrl->KV.x = REMOTE.RC_ctrl->KV.y = REMOTE.RC_ctrl->KV.z = 0;
    REMOTE.RC_ctrl->KV.key = REMOTE.RC_ctrl->KV.press_l = REMOTE.RC_ctrl->KV.press_r = 0;



    /*��ͨ�˲���ʼ��*/
    First_Order_Init(&REMOTE.RC_X, 0.08);
    First_Order_Init(&REMOTE.RC_Y, 0.08);
    First_Order_Init(&REMOTE.RC_Z, 0.08);

    First_Order_Init(&REMOTE.KM_X, 0.005);
    First_Order_Init(&REMOTE.KM_Y, 0.005);
    First_Order_Init(&REMOTE.KM_Z, 0.005);

    /*��ȡң��ָ��*/
    REMOTE.RC_ctrl = REMOTE.Get_Remote_Point();
}



/*************************************************************************************************
*����:	Rc_Deal
*����:	����ң�˲��˵�ֵ
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
static void Rc_Deal(void)
{
    /*��������*/
    REMOTE.RC_ctrl->ch0 = Dead_Zone(REMOTE.RC_ctrl->ch0, DeadZone);
    REMOTE.RC_ctrl->ch1 = Dead_Zone(REMOTE.RC_ctrl->ch1, DeadZone);
    REMOTE.RC_ctrl->ch2 = Dead_Zone(REMOTE.RC_ctrl->ch2, DeadZone);

    /*�˲�����*/
    First_Order(&REMOTE.RC_X, REMOTE.RC_ctrl->ch1);
    First_Order(&REMOTE.RC_Y, REMOTE.RC_ctrl->ch0);
    First_Order(&REMOTE.RC_Z, REMOTE.RC_ctrl->ch2);
}



/*************************************************************************************************
*����:	KEY_MOUSE
*����:	�����������ֵ
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
static void Key_Mouse_Deal(void)
{
    uint16_t key = REMOTE.RC_ctrl->KV.key;

    /*ǰ��*/
    if(key & KEY_PRESSED_OFFSET_W)
    {
        REMOTE.RC_ctrl->KV.kv0 += 10;
    }
    else if(key & KEY_PRESSED_OFFSET_S)
    {
        REMOTE.RC_ctrl->KV.kv0 -= 10;
    }
    else
    {
        REMOTE.RC_ctrl->KV.kv0 = 0;
    }

    /*����*/
    if(key & KEY_PRESSED_OFFSET_A)
    {
        REMOTE.RC_ctrl->KV.kv1 -= 10;
    }
    else if(key & KEY_PRESSED_OFFSET_D)
    {
        REMOTE.RC_ctrl->KV.kv1 += 10;
    }
    else
    {
        REMOTE.RC_ctrl->KV.kv1 = 0;
    }

    /*Y����ת*/
    REMOTE.RC_ctrl->KV.kv3 = (float)REMOTE.RC_ctrl->KV.x * 15;

    /*Z����ת*/
    REMOTE.RC_ctrl->KV.kv2 = (float)REMOTE.RC_ctrl->KV.y * 15;

    /*���Ʒ��ȴ���*/
    REMOTE.RC_ctrl->KV.kv0 = limit(REMOTE.RC_ctrl->KV.kv0, 660, -660);
    REMOTE.RC_ctrl->KV.kv1 = limit(REMOTE.RC_ctrl->KV.kv1, 660, -660);
    REMOTE.RC_ctrl->KV.kv2 = limit(REMOTE.RC_ctrl->KV.kv2, 660, -660);
    REMOTE.RC_ctrl->KV.kv3 = limit(REMOTE.RC_ctrl->KV.kv3, 660, -660);

    /*�˲�����*/
    First_Order(&REMOTE.KM_X, REMOTE.RC_ctrl->KV.kv1);
    First_Order(&REMOTE.KM_Y, REMOTE.RC_ctrl->KV.kv0);
    First_Order(&REMOTE.KM_Z, REMOTE.RC_ctrl->KV.kv2);

}



/*************************************************************************************************
*����:	RC_DATA_DEAL
*����:	ң����ֵ����
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
void Remote_Data_Deal(void)
{
    /*���ݸ��±�־λ*/
    if(REMOTE.RC_ctrl->Flag == 1)
    {
        Rc_Deal();
    }
}









