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
*����:	Remote_Data_Zero
*����:	ң����ֵ����
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
static void Remote_Data_Zero(void)
{
    /*ң����ֵ��ʼ��*/
    REMOTE.RC_ctrl->ch0 = REMOTE.RC_ctrl->ch1 = REMOTE.RC_ctrl->ch2 = REMOTE.RC_ctrl->ch3 = 0;
    REMOTE.RC_ctrl->sw = REMOTE.RC_ctrl->Flag = 0;
    REMOTE.RC_ctrl->KV.x = REMOTE.RC_ctrl->KV.y = REMOTE.RC_ctrl->KV.z = 0;
    REMOTE.RC_ctrl->KV.key = REMOTE.RC_ctrl->KV.press_l = REMOTE.RC_ctrl->KV.press_r = 0;

    /*״ֵ̬��ʼ�� ȫ����ͣ*/
				REMOTE.state.Grasp_Up = 0; //
				REMOTE.state.Translation = 1;  //
				REMOTE.state.Telescoping = 0;
				REMOTE.state.Clap = 0;
				REMOTE.state.Flip = 0;
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
    /*ң����ֵ����*/
    Remote_Data_Zero();
    REMOTE.RC_ctrl->s1 = REMOTE.RC_ctrl->s2 = 2; //�˼�λΪ�ϵ��λ

    /*��ͨ�˲���ʼ��*/
    First_Order_Init(&REMOTE.RC_X, 0.08);
    First_Order_Init(&REMOTE.RC_Y, 0.08);
    First_Order_Init(&REMOTE.RC_Z, 0.08);

    First_Order_Init(&REMOTE.KM_X, 0.08);
    First_Order_Init(&REMOTE.KM_Y, 0.08);
    First_Order_Init(&REMOTE.KM_Z, 0.08);

    /*��ȡң��ָ��*/
    REMOTE.RC_ctrl = Return_Remote_Point();
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
*˵��:	��֧�ṹ�����Ż�
*************************************************************************************************/
static void Key_Mouse_Deal(void)
{
    uint16_t key = REMOTE.RC_ctrl->KV.key;

    /*************************************����ǰ��*********************************/
    if (key & KEY_PRESSED_OFFSET_W)
    {
        if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_W)
        {
            REMOTE.RC_ctrl->KV.kv0 = 6000;
        }
        else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_W)
        {
            REMOTE.RC_ctrl->KV.kv0 = 200;
        }
        else
        {
            REMOTE.RC_ctrl->KV.kv0 = 3000;
        }
    }
    else if (key & KEY_PRESSED_OFFSET_S)
    {
        if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_S)
        {
            REMOTE.RC_ctrl->KV.kv0 = -6000;
        }
        else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_S)
        {
            REMOTE.RC_ctrl->KV.kv0 = -200;
        }
        else
        {
            REMOTE.RC_ctrl->KV.kv0 = -3000;
        }
    }
    else
    {
        REMOTE.RC_ctrl->KV.kv0 = 0;
    }

    /**************************************��������*********************************/
    if (key & KEY_PRESSED_OFFSET_A)
    {
        if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_A)
        {
            REMOTE.RC_ctrl->KV.kv1 = -6000;
        }
        else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_A)
        {
            REMOTE.RC_ctrl->KV.kv1 = -200;
        }
        else
        {
            REMOTE.RC_ctrl->KV.kv1 = -3000;
        }
    }
    else if (key & KEY_PRESSED_OFFSET_D)
    {
        if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_D)
        {
            REMOTE.RC_ctrl->KV.kv1 = 6000;
        }
        else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_D)
        {
            REMOTE.RC_ctrl->KV.kv1 = 200;
        }
        else
        {
            REMOTE.RC_ctrl->KV.kv1 = 3000;
        }
    }
    else
    {
        REMOTE.RC_ctrl->KV.kv1 = 0;
    }

    /*************************************���X��************************************/
    REMOTE.RC_ctrl->KV.kv3 = (float)REMOTE.RC_ctrl->KV.x * 5.0f;
    /**************************************���Y��***********************************/
    REMOTE.RC_ctrl->KV.kv2 = (float)REMOTE.RC_ctrl->KV.y * 5.0f;

    /*******************************�Զ���ȡ********************************/
    if (key & KEY_PRESSED_OFFSET_Q)
    {
            REMOTE.state.Grasp_Up = 1; // �Զ���ȡ
    }

    /********************************�һ�̧��*******************************/
    if (key & KEY_PRESSED_OFFSET_E)
    {
            REMOTE.state.Grasp_Up = 2; // �Զ���ȡ
    }

    /********************************��ת��λ*******************************/
    if (key & KEY_PRESSED_OFFSET_R)
    {
            REMOTE.state.Grasp_Up = 3; // �Զ���ȡ
    }

    /*********************************RFID********************************/
    if (key & KEY_PRESSED_OFFSET_F)
    {

        if (key & KEY_PRESSED_OFFSET_F && key & KEY_PRESSED_OFFSET_CTRL)
        {
            REMOTE.state.Gimbal_Yaw = 1;
        }
        else
        {
            REMOTE.state.Gimbal_Yaw = 0;
        }
    }

    /*********************************��Ԯצ********************************/
    if (key & KEY_PRESSED_OFFSET_G)
    {

        if (key & KEY_PRESSED_OFFSET_G && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.RFID = 0; //RFID��
            REMOTE.state.Barrier = 0; //��Ԯצ��
        }
        else
        {
						REMOTE.state.RFID = 1; //RFID��
            REMOTE.state.Barrier = 1; //��Ԯצ��
        }
    }

    /* ========================================����Ϊ����====================================== */

    /*********************************����**********************************/
    if (key & KEY_PRESSED_OFFSET_SHIFT)
    {

        REMOTE.state.Independent = 1; //���������˶�
        REMOTE.state.Rotation = 2;    //С����ģʽ��ͣ
        REMOTE.state.Wiggle = 2;      //Ť��ģʽ��ͣ
    }

    /*********************************������**********************************/
    if (key & KEY_PRESSED_OFFSET_Z)
    {

        REMOTE.state.Independent = 2; //����������ͣ
        REMOTE.state.Rotation = 1;    //������ģʽ
        REMOTE.state.Wiggle = 2;      //Ť��ģʽ��ͣ
    }
		
    /* ======================================����Ϊ����===================================== */
    /**********************************�ɼ�**********************************/
    if (key & KEY_PRESSED_OFFSET_X)
    {
        if (key & KEY_PRESSED_OFFSET_X && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.Translation = 0;  //��װ��λ�ر�
        }
        else
        {
						REMOTE.state.Translation = 1;  //��װ��λ�ر�
				}
    }

    /*******************************�һ�һ��********************************/
    if (key & KEY_PRESSED_OFFSET_C)
    {
        if (key & KEY_PRESSED_OFFSET_C && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.Telescoping = 0;  //��װ��λ�ر�
        }
        else
        {
						REMOTE.state.Telescoping = 1;  //��װ��λ�ر�
				}
    }

    /********************************�һ�����********************************/
    if (key & KEY_PRESSED_OFFSET_V)
    {
        if (key & KEY_PRESSED_OFFSET_V && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.Clap = 0;  //��װ��λ�ر�
        }
        else
        {
						REMOTE.state.Clap = 1;  //��װ��λ�ر�
				}
    }

    /********************************�һ�����********************************/
    if (key & KEY_PRESSED_OFFSET_B)
    {
        if (key & KEY_PRESSED_OFFSET_B && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.Flip = 0;  //��װ��λ�ر�
        }
        else
        {
						REMOTE.state.Flip = 1;  //��װ��λ�ر�
				}
    }

    /*���Ʒ��ȴ���*/
    REMOTE.RC_ctrl->KV.kv0 = limit(REMOTE.RC_ctrl->KV.kv0, 660, -660);
    REMOTE.RC_ctrl->KV.kv1 = limit(REMOTE.RC_ctrl->KV.kv1, 660, -660);
    REMOTE.RC_ctrl->KV.kv2 = limit(REMOTE.RC_ctrl->KV.kv2, 660, -660);
    REMOTE.RC_ctrl->KV.kv3 = limit(REMOTE.RC_ctrl->KV.kv3, 660, -660);
    /*�˲�����*/
    First_Order(&REMOTE.KM_X, REMOTE.RC_ctrl->KV.kv0); //��������
    First_Order(&REMOTE.KM_Y, REMOTE.RC_ctrl->KV.kv1); //����ǰ��
    First_Order(&REMOTE.KM_Z, REMOTE.RC_ctrl->KV.kv3); // ���x��
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
    static unsigned char _s1 = 0;

    if (_s1 != REMOTE.RC_ctrl->s1)
        Remote_Data_Zero();

    if (REMOTE.RC_ctrl->Flag)
    {
        if (REMOTE.RC_ctrl->s1 == 3)
        {
            Key_Mouse_Deal();
        }
        else
        {
            Rc_Deal();
        }

        REMOTE.RC_ctrl->Flag = 0;
    }

    _s1 = REMOTE.RC_ctrl->s1;
}
