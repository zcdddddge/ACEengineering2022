#include "encoder.h"


/*************************************************************************************************
*名称:	CAN_DATA_Encoder_Deal
*功能:	u8 radio -> 减速比		int16_t CanData->数据		Encoder_t * Encoder->码盘结构体
*形参: 	无
*返回:	无
*说明:	码盘值处理
*************************************************************************************************/
void CAN_DATA_Encoder_Deal(u8 radio, int16_t CanData, int16_t speed, Encoder_t * Encoder)
{
    u8 Radio = radio;

    /*速度处理*/
    Encoder->Speed[1] = speed;
    Encoder->AccSpeed	=	Encoder->Speed[1] - Encoder->Speed[0];
    Encoder->Speed[0] = Encoder->Speed[1];

    /*码盘状态判断*/
    if(CanData <= 0 || CanData > 8192)
    {
        Encoder->State = WRONG;
    }
    else
    {
        Encoder->State = NORM;
    }

    /*码盘处理*/
    /*零界点判断*/
    if(CanData < 7000 && CanData > 2000)
    {
        /*码盘值在一小圈内*/
        Encoder->Ahead[0] = 0;
        Encoder->Ahead[1] = 0;
        Encoder->Back[0] = 0;
        Encoder->Back[1] = 0;
    }
    else if(CanData > 7000)
    {
        /*码盘值正向接近一小圈*/
        Encoder->Ahead[0] = 1;
    }
    else if(CanData < 2000)
    {
        /*码盘值反向接近一小圈*/
        Encoder->Back[0] = 1;
    }


    /*越过零界点判断*/
    if(Encoder->Ahead[0] == 1 && CanData < 2000)
    {
        /*码盘值正向越过一小圈*/
        Encoder->Ahead[1] = 1;
    }
    else if(Encoder->Back[0] == 1 && CanData > 7000)
    {
        /*码盘值反向越过一小圈*/
        Encoder->Back[1] = 1;
    }


    /*零界点处理*/
    if(Encoder->Ahead[0] == 1 && Encoder->Ahead[1] == 1)
    {
        Encoder->Radio_Circle ++; 							//累加一小圈
        Encoder->Encode_Record_Val += 8192;			//累加码盘值
        Encoder->Ahead[0] = 0;									//标志位清零
        Encoder->Ahead[1] = 0;									//标志位清零
    }

    else if(Encoder->Back[0] == 1 && Encoder->Back[1] == 1)
    {
        Encoder->Radio_Circle --;								//累减一小圈
        Encoder->Encode_Record_Val -= 8192;			//累减码盘值
        Encoder->Back[0] = 0;										//标志位清零
        Encoder->Back[1] = 0;										//标志位清零
    }

    if(Encoder->Radio_Circle > 0)
    {
        Encoder->Encode_Actual_Val = Encoder->Radio_Circle * 8192 + CanData;				 	//码盘一大圈内的真实码盘值
    }
    else if(Encoder->Radio_Circle < 0)
    {
        Encoder->Encode_Actual_Val = 8192 * Radio + (Encoder->Radio_Circle - 1) * 8192 + CanData;		//码盘一大圈内的真实码盘值
    }
    else if(Encoder->Radio_Circle == 0)
    {
        if(Encoder->Encode_Record_Val > 8192)																				//反向跨越
        {
            Encoder->Encode_Actual_Val = 8192 * Radio + (Encoder->Radio_Circle - 1) * 8192 + CanData;;
        }
        else if(Encoder->Encode_Record_Val <= 8192)																	//正向跨越
        {
            Encoder->Encode_Actual_Val = CanData;
        }
    }

    /*电机一大圈处理*/
    /*电机正向转过一大圈*/
    if(Encoder->Encode_Record_Val > (8192 * Radio))
    {
        Encoder->Encode_Record_Val = CanData;											//码盘记录值为当前值
        Encoder->Actual_Circle ++;																//电机实际圈数累加
        Encoder->Radio_Circle = 0;  															//清零小圈
    }
    /*电机反向转过一大圈*/
    else if(Encoder->Encode_Record_Val < 0)
    {
        Encoder->Encode_Record_Val = 8192 * (Radio - 1) + CanData;	//码盘记录值复位
        Encoder->Actual_Circle --;																//码盘一大圈累减
        Encoder->Radio_Circle = 0;  															//清零小圈
    }
    else
    {
        Encoder->Radian = (360.0 / (Radio * 8192) * (Encoder->Encode_Actual_Val));	//一大圈角度
        Encoder->Total_Radian = Encoder->Actual_Circle * 360.0f + Encoder->Radian;
    }



    if(Encoder->State == WRONG)
    {

    }

    else if(Encoder->State == BLOCK)
    {

    }

}



/*码盘值数值清零处理*/
void EncoderValZero(Encoder_t * Encoder)
{
    Encoder->Ahead[0] = Encoder->Ahead[1] = Encoder->Back[0] = Encoder->Back[1] = 0;
    Encoder->Actual_Circle = Encoder->Radio_Circle = Encoder->Encode_Actual_Val = Encoder->Encode_Record_Val = 0;
    Encoder->Radian = Encoder->Init_Radian = Encoder->Lock_Radian = Encoder->Total_Radian = 0.0;
    Encoder->State = WRONG;
    Encoder->Speed[0] = Encoder->Speed[1] = Encoder->AccSpeed = 0;
}
