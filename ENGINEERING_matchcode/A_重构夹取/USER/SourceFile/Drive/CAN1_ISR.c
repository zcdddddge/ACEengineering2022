#include "CAN1_ISR.h"


/*数据接收区*/
int16_t can1_201_208_buffer[16];


/*码盘值结构体*/
Encoder_t can1_encoder_201;
Encoder_t can1_encoder_202;
Encoder_t can1_encoder_203;
Encoder_t can1_encoder_204;
Encoder_t can1_encoder_205;
Encoder_t can1_encoder_206;
Encoder_t can1_encoder_207;
Encoder_t can1_encoder_208;


/*************************************************************************************************
*名称:	CAN1_RX0_IRQHandler
*功能:	can1 接收函数
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

        switch(RxMessage.StdId)
        {
            case 0x201:
            {
                can1_201_208_buffer[0] = (RxMessage.Data[0] << 8) + RxMessage.Data[1];
                can1_201_208_buffer[1] = (RxMessage.Data[2] << 8) + RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(27, can1_201_208_buffer[0], can1_201_208_buffer[1], &can1_encoder_201);
                break;
            }

            case 0x202:
            {
                can1_201_208_buffer[2] = (RxMessage.Data[0] << 8) + RxMessage.Data[1];
                can1_201_208_buffer[3] = (RxMessage.Data[2] << 8) + RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(19, can1_201_208_buffer[2], can1_201_208_buffer[3], &can1_encoder_202);
                break;
            }

            case 0x203:
            {
                can1_201_208_buffer[4] = (RxMessage.Data[0] << 8) + RxMessage.Data[1];
                can1_201_208_buffer[5] = (RxMessage.Data[2] << 8) + RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(36, can1_201_208_buffer[4], can1_201_208_buffer[5], &can1_encoder_203);
                break;
            }

            case 0x204:
            {
                can1_201_208_buffer[6] = (RxMessage.Data[0] << 8) + RxMessage.Data[1];
                can1_201_208_buffer[7] = (RxMessage.Data[2] << 8) + RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(27, can1_201_208_buffer[6], can1_201_208_buffer[7], &can1_encoder_204);
                break;
            }

            case 0x205:
            {
                can1_201_208_buffer[8] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
                can1_201_208_buffer[9] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(19, can1_201_208_buffer[8], can1_201_208_buffer[9], &can1_encoder_205);
                break;
            }

            case 0x206:
            {
                can1_201_208_buffer[10] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
                can1_201_208_buffer[11] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(19, can1_201_208_buffer[10], can1_201_208_buffer[11], &can1_encoder_206);
                break;
            }

            case 0x207:
            {
                can1_201_208_buffer[12] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
                can1_201_208_buffer[13] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(36, can1_201_208_buffer[12], can1_201_208_buffer[13], &can1_encoder_207);
                break;
            }

            case 0x208:
            {
                can1_201_208_buffer[14] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
                can1_201_208_buffer[15] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(45, can1_201_208_buffer[14], can1_201_208_buffer[15], &can1_encoder_208);

                break;
            }

            case 0x301:
            {
                break;
            }

            default:
                break;
        }
    }
}




/*************************************************************************************************
*名称:	CAN1_201_To_204_SEND
*功能:	can1 发送函数
*形参: 	int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204
*返回:	无
*说明:	无
*************************************************************************************************/
void CAN1_201_To_204_SEND(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg TxMessage;     									//定义一个发送信息的结构体

    TxMessage.StdId = 0x200;      						//根据820r设置标识符
    TxMessage.IDE = CAN_ID_STD;   						//指定将要传输的消息的标识符的类型
    TxMessage.RTR = CAN_RTR_DATA; 						//指定的帧将被传输的消息的类型   数据帧或远程帧
    TxMessage.DLC = 8;      									//指定数据的长度
    TxMessage.Data[0] = (unsigned char)(ESC_201 >> 8);
    TxMessage.Data[1] = (unsigned char)(ESC_201);
    TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    TxMessage.Data[3] = (unsigned char)(ESC_202);
    TxMessage.Data[4] = (unsigned char)(ESC_203 >> 8);
    TxMessage.Data[5] = (unsigned char)(ESC_203);
    TxMessage.Data[6] = (unsigned char)(ESC_204 >> 8);
    TxMessage.Data[7] = (unsigned char)(ESC_204);
    mbox = CAN_Transmit(CAN1, &TxMessage);  			//发送信息
    i = 0;

    while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}




/*************************************************************************************************
*名称:	CAN1_205_To_208_SEND
*功能:	can1 发送函数
*形参: 	int16_t control_205,int16_t control_206, int16_t control_207, int16_t control_208
*返回:	无
*说明:	无
*************************************************************************************************/
void CAN1_205_To_208_SEND(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x1ff;	 // 标准标识符为0
    TxMessage.IDE = 0;		   // 使用扩展标识符
    TxMessage.RTR = 0;		   // 消息类型为数据帧，一帧8位
    TxMessage.DLC = 8;		   // 发送两帧信息
    TxMessage.Data[0] = (unsigned char)(ESC_205 >> 8);
    TxMessage.Data[1] = (unsigned char)(ESC_205);
    TxMessage.Data[2] = (unsigned char)(ESC_206 >> 8);
    TxMessage.Data[3] = (unsigned char)(ESC_206);
    TxMessage.Data[4] = (unsigned char)(ESC_207 >> 8);
    TxMessage.Data[5] = (unsigned char)(ESC_207);
    TxMessage.Data[6] = (unsigned char)(ESC_208 >> 8);
    TxMessage.Data[7] = (unsigned char)(ESC_208);

    mbox = CAN_Transmit(CAN1, &TxMessage);
    i = 0;

    while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))i++;	//等待发送结束
}




/*************************************************************************************************
*名称:	CAN1_To_Board
*功能:	发送用户发送自定义数据
*形参: 	unsigned char*CAN_DATA , int16_t stdid
*返回:	无
*说明:	自己定义标识符
*************************************************************************************************/
void CAN1_To_Board(u8*CAN_DATA, int16_t stdid)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg TxMessage;     									//定义一个发送信息的结构体

    TxMessage.StdId = stdid;      						//根据820r设置标识符
    TxMessage.IDE = CAN_ID_STD;   						//指定将要传输的消息的标识符的类型
    TxMessage.RTR = CAN_RTR_DATA; 						//指定的帧将被传输的消息的类型   数据帧或远程帧
    TxMessage.DLC = 8;      									//指定数据的长度
    TxMessage.Data[0] = (unsigned char)(CAN_DATA[0]);
    TxMessage.Data[1] = (unsigned char)(CAN_DATA[1]);
    TxMessage.Data[2] = (unsigned char)(CAN_DATA[2]);
    TxMessage.Data[3] = (unsigned char)(CAN_DATA[3]);
    TxMessage.Data[4] = (unsigned char)(CAN_DATA[4]);
    TxMessage.Data[5] = (unsigned char)(CAN_DATA[5]);
    TxMessage.Data[6] = (unsigned char)(CAN_DATA[6]);
    TxMessage.Data[7] = (unsigned char)(CAN_DATA[7]);
    mbox = CAN_Transmit(CAN1, &TxMessage);  			//发送信息
    i = 0;

    while((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}



/*************************************************************************************************
*名称:	Return_Can1_201_208_Encoder
*功能:	返回Can1_201_208数据处理指针
*形参: 	u8 ch
*返回:	结构体地址
*说明:	无
*************************************************************************************************/
Encoder_t * Return_Can1_201_208_Encoder(u8 ch)
{
    switch(ch)
    {
        case 1:
        {
            return &can1_encoder_201;
        }

        case 2:
        {
            return &can1_encoder_202;
        }

        case 3:
        {
            return &can1_encoder_203;
        }

        case 4:
        {
            return &can1_encoder_204;
        }

        case 5:
        {
            return &can1_encoder_205;
        }

        case 6:
        {
            return &can1_encoder_206;
        }

        case 7:
        {
            return &can1_encoder_207;
        }

        case 8:
        {
            return &can1_encoder_208;
        }

        default:
            break;
    }

    return 0;
}


