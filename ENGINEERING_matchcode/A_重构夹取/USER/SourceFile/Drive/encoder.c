#include "encoder.h"


/*************************************************************************************************
*����:	CAN_DATA_Encoder_Deal
*����:	u8 radio -> ���ٱ�		int16_t CanData->����		Encoder_t * Encoder->���̽ṹ��
*�β�: 	��
*����:	��
*˵��:	����ֵ����
*************************************************************************************************/
void CAN_DATA_Encoder_Deal(u8 radio, int16_t CanData, int16_t speed, Encoder_t * Encoder)
{
    u8 Radio = radio;

    /*�ٶȴ���*/
    Encoder->Speed[1] = speed;
    Encoder->AccSpeed	=	Encoder->Speed[1] - Encoder->Speed[0];
    Encoder->Speed[0] = Encoder->Speed[1];

    /*����״̬�ж�*/
    if(CanData <= 0 || CanData > 8192)
    {
        Encoder->State = WRONG;
    }
    else
    {
        Encoder->State = NORM;
    }

    /*���̴���*/
    /*�����ж�*/
    if(CanData < 7000 && CanData > 2000)
    {
        /*����ֵ��һСȦ��*/
        Encoder->Ahead[0] = 0;
        Encoder->Ahead[1] = 0;
        Encoder->Back[0] = 0;
        Encoder->Back[1] = 0;
    }
    else if(CanData > 7000)
    {
        /*����ֵ����ӽ�һСȦ*/
        Encoder->Ahead[0] = 1;
    }
    else if(CanData < 2000)
    {
        /*����ֵ����ӽ�һСȦ*/
        Encoder->Back[0] = 1;
    }


    /*Խ�������ж�*/
    if(Encoder->Ahead[0] == 1 && CanData < 2000)
    {
        /*����ֵ����Խ��һСȦ*/
        Encoder->Ahead[1] = 1;
    }
    else if(Encoder->Back[0] == 1 && CanData > 7000)
    {
        /*����ֵ����Խ��һСȦ*/
        Encoder->Back[1] = 1;
    }


    /*���㴦��*/
    if(Encoder->Ahead[0] == 1 && Encoder->Ahead[1] == 1)
    {
        Encoder->Radio_Circle ++; 							//�ۼ�һСȦ
        Encoder->Encode_Record_Val += 8192;			//�ۼ�����ֵ
        Encoder->Ahead[0] = 0;									//��־λ����
        Encoder->Ahead[1] = 0;									//��־λ����
    }

    else if(Encoder->Back[0] == 1 && Encoder->Back[1] == 1)
    {
        Encoder->Radio_Circle --;								//�ۼ�һСȦ
        Encoder->Encode_Record_Val -= 8192;			//�ۼ�����ֵ
        Encoder->Back[0] = 0;										//��־λ����
        Encoder->Back[1] = 0;										//��־λ����
    }

    if(Encoder->Radio_Circle > 0)
    {
        Encoder->Encode_Actual_Val = Encoder->Radio_Circle * 8192 + CanData;				 	//����һ��Ȧ�ڵ���ʵ����ֵ
    }
    else if(Encoder->Radio_Circle < 0)
    {
        Encoder->Encode_Actual_Val = 8192 * Radio + (Encoder->Radio_Circle - 1) * 8192 + CanData;		//����һ��Ȧ�ڵ���ʵ����ֵ
    }
    else if(Encoder->Radio_Circle == 0)
    {
        if(Encoder->Encode_Record_Val > 8192)																				//�����Խ
        {
            Encoder->Encode_Actual_Val = 8192 * Radio + (Encoder->Radio_Circle - 1) * 8192 + CanData;;
        }
        else if(Encoder->Encode_Record_Val <= 8192)																	//�����Խ
        {
            Encoder->Encode_Actual_Val = CanData;
        }
    }

    /*���һ��Ȧ����*/
    /*�������ת��һ��Ȧ*/
    if(Encoder->Encode_Record_Val > (8192 * Radio))
    {
        Encoder->Encode_Record_Val = CanData;											//���̼�¼ֵΪ��ǰֵ
        Encoder->Actual_Circle ++;																//���ʵ��Ȧ���ۼ�
        Encoder->Radio_Circle = 0;  															//����СȦ
    }
    /*�������ת��һ��Ȧ*/
    else if(Encoder->Encode_Record_Val < 0)
    {
        Encoder->Encode_Record_Val = 8192 * (Radio - 1) + CanData;	//���̼�¼ֵ��λ
        Encoder->Actual_Circle --;																//����һ��Ȧ�ۼ�
        Encoder->Radio_Circle = 0;  															//����СȦ
    }
    else
    {
        Encoder->Radian = (360.0 / (Radio * 8192) * (Encoder->Encode_Actual_Val));	//һ��Ȧ�Ƕ�
        Encoder->Total_Radian = Encoder->Actual_Circle * 360.0f + Encoder->Radian;
    }



    if(Encoder->State == WRONG)
    {

    }

    else if(Encoder->State == BLOCK)
    {

    }

}



/*����ֵ��ֵ���㴦��*/
void EncoderValZero(Encoder_t * Encoder)
{
    Encoder->Ahead[0] = Encoder->Ahead[1] = Encoder->Back[0] = Encoder->Back[1] = 0;
    Encoder->Actual_Circle = Encoder->Radio_Circle = Encoder->Encode_Actual_Val = Encoder->Encode_Record_Val = 0;
    Encoder->Radian = Encoder->Init_Radian = Encoder->Lock_Radian = Encoder->Total_Radian = 0.0;
    Encoder->State = WRONG;
    Encoder->Speed[0] = Encoder->Speed[1] = Encoder->AccSpeed = 0;
}
