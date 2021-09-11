/*************************************************************************************************
ģ������ : UserReferee
ʵ�ֹ��� : �����û��Զ�������������ϵͳ
��    ע : ��
-------------------------------------------------------------------------------------------------
���� : 20-03-03
�汾 : 1.0
���� : ��
˵�� : ͨ��
-------------------------------------------------------------------------------------------------
���� : 20-04-10
�汾 : 1.1
���� : �����û����з��Ͳ���
˵�� : �������˼�ͨ����Ҫ�õ�������������Ҫ��ʼ�����Ի�ȡ����ϵͳ����ָ��
*************************************************************************************************/

#include "UserReferee.h"
#include "REFEREE_ISR.h"
#include <stdlib.h>
#include "rt_heap.h"


/*����ϵͳ���ݽṹ��*/
REFEREE_t *UserReffee;
/*��ʼ��*/
void UserDataInit(void)
{
    UserReffee = Return_Referee_Point();
}


/*************************************************************************************************
*����:	SendUserData
*����:	�����û��Զ�������������ϵͳ
*�β�: 	DataLen�����ݶγ���
				seq�������0-255
				ContentID���Զ�����������
				SendID��������ID
				ReceiveID��������ID
				data������
				user_data_t���û����ݽṹ��
*����:	1 ���� ʧ��
				0 ���� �ɹ�
*˵��:	ͨ��
				ʵ����Ҫ����
*************************************************************************************************/
u8 SendUserData(uint16_t DataLen, u8 seq, uint16_t ContentID, uint16_t SendID, uint16_t ReceiveID, u8 *data)
{
    static int16_t CmdID = 0x0301;
    uint16_t i = 0;
    uint16_t Clock = 0;
    /*BCDתʮ����*/
    uint16_t Len = ( ((DataLen >> 8) * 100) + ( ((DataLen >> 4) & 0x0f) * 10 ) + (DataLen & 0x0f) );
    /*����ռ�*/
    u8*user = (u8*)malloc( (sizeof(u8)) * (15 + Len) );

    if(user == NULL)
    {
        return 1;
    }

    /*֡ͷ*/
    user[0] = 0xA5;
    user[1] = DataLen;
    user[2] = (DataLen >> 8);
    user[3] = seq;
    /*CRC-8*/
    Append_CRC8_Check_Sum(user, 5);

    /*�Զ����������*/
    user[5] = CmdID;
    user[6] = (CmdID >> 8);
    user[7] = ContentID;
    user[8] = (ContentID >> 8);
    user[9] = SendID;
    user[10] = (SendID >> 8);
    user[11] = ReceiveID;
    user[12] = (ReceiveID >> 8);

    /*�Զ�������*/
    for(i = 13; i < (13 + Len); i ++)
        user[i] = data[i - 13];

    /*CRC-16*/
    Append_CRC16_Check_Sum(user, (15 + Len));

    /*����*/
    for(i = 0; i < (15 + Len); i ++)
    {
        Clock = 0;

        while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
        {
            Clock ++;

            if(Clock >= 0xf1f1)
            {
                free(user);
                user = NULL;
                return 1;
            }
        }

        USART_SendData(USART6, user[i]);
    }

    free(user);
    user = NULL;
    return 0;
}



/*************************************************************************************************
*����:	SendPowerHeat
*����:	���͹����������ͻ���
*�β�:	SendID	              �������˶�ID
				ReceiveID						  ����Ӧ�ͻ���ID
*����:	1 ���� ʧ��
				0 ���� �ɹ�
*˵��:	1. ������ ID��1��Ӣ��(��)��2������(��)��3/4/5������(��)��6������(��)��7���ڱ�(��)��11��Ӣ��(��)��
					 12������(��)��13/14/15������(��)��16������(��)��17���ڱ�(��)��
				2. �ͻ��� ID��0x0101 ΪӢ�۲����ֿͻ���(��)��0x0102�����̲����ֿͻ���((��)��
					 0x0103/0x0104/0x0105�����������ֿͻ���(��)��0x0106�����в����ֿͻ���((��)�� 0x0111��Ӣ�۲����ֿͻ���(��)��
					 0x0112�����̲����ֿͻ���(��)��0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��0x0116�����в����ֿͻ���(��)
*************************************************************************************************/
u8 SendPowerHeat(uint16_t SendID, uint16_t ReceiveID)
{
    u8 data[13];
    Float_t p;
    p.value = UserReffee->PowerHeat.chassis_power;
    data[0] = p.float_byte.LB;
    data[1] = p.float_byte.MLB;
    data[2] = p.float_byte.MHB;
    data[3] = p.float_byte.HB;
    p.value = 0.0f;
    data[4] = p.float_byte.LB;
    data[5] = p.float_byte.MLB;
    data[6] = p.float_byte.MHB;
    data[7] = p.float_byte.HB;
    p.value = 0.0f;
    data[8] = p.float_byte.LB;
    data[9] = p.float_byte.MLB;
    data[10] = p.float_byte.MHB;
    data[11] = p.float_byte.HB;
    data[12] = 0x15;

    if(SendUserData(0x13, 0x01, 0xD180, SendID, ReceiveID, data) == 0)
        return 0;
    else
        return 1;
}



/*************************************************************************************************
*����:	RobotCommuni
*����:	�����˼�ͨ��
*�β�: 	Len              �����ݶγ���
				ContentID        ��0x0200~0x02FF
				SendID           ��������ID
				ReceiveID        ��������ID
				data             ������
*����:	1 ���� ʧ��
				0 ���� �ɹ�
*˵��:	���ݶ�ID��ò�Ҫ��ͻ
*************************************************************************************************/
u8 RobotCommuni(uint16_t SendID, uint16_t ReceiveID, u8 Len, uint16_t ContentID, u8*data)
{
    /*ʮ����תBCD��*/
    uint16_t DataLen = ( ((Len / 100) << 8) | ((Len / 10 % 10) << 4) | (Len % 10) );

    if(SendUserData(DataLen, 0x02, ContentID, SendID, ReceiveID, data) == 0)
        return 0;
    else
        return 1;
}



/*************************************************************************************************
*����:	DeleteAllMap
*����:	ɾ�������Զ���ͼ��
*�β�: 	SendID           ��������ID
				ReceiveID        ��������ID
*����:	1 ���� ʧ��
				0 ���� �ɹ�
*˵��:	��
*************************************************************************************************/
u8 DeleteAllMap(uint16_t SendID, uint16_t ReceiveID)
{
    u8 data[55] = {0};
    data[0] = 0x06;

    if(SendUserData(0x55, 0x03, 0x0100, SendID, ReceiveID, data) == 0)
        return 0;
    else
        return 1;
}



/*************************************************************************************************
*����:	ClientMap
*����:	�Զ���ͼ��
*�β�: 	SendID           ��������ID
				ReceiveID        ��������ID
				ClientMap_t			 ��ͼ�νṹ��
*����:	1 ���� ʧ��
				0 ���� �ɹ�
*˵��:	��
*************************************************************************************************/
u8 ClientMap(uint16_t SendID, uint16_t ReceiveID, ClientMap_t*user)
{
    u8 data[55] = {0};
    u8 i = 0;

    data[0] = user->operate_tpye;
    data[1] = user->graphic_tpye;

    for(i = 0; i < 5; i ++)
        data[i + 2] = user->graphic_name[i];

    data[7] = user->layer;
    data[8] = user->color;
    data[9] = user->width;
    data[10] = user->start_x;
    data[11] = (user->start_x >> 8);
    data[12] = user->start_y;
    data[13] = (user->start_y >> 8);
    data[14] = user->radius;
    data[15] = (user->radius >> 8);
    data[16] = user->end_x;
    data[17] = (user->end_x >> 8);
    data[18] = user->end_y;
    data[19] = (user->end_y >> 8);
    data[20] = user->start_angle;
    data[21] = (user->start_angle >> 8);
    data[22] = user->end_angle;
    data[23] = (user->end_angle >> 8);
    i =  ( ((user->text_lenght / 10) << 4) | (user->text_lenght % 10) );
    data[24] = i;

    for(i = 0; i < user->text_lenght; i ++)
        data[25 + i] = user->text[i];

    if(SendUserData(0x55, 0x04, 0x0100, SendID, ReceiveID, data) == 0)
        return 0;
    else
        return 1;
}
