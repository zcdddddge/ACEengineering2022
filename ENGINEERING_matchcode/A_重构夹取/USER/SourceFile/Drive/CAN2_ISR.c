#include "CAN2_ISR.h"

/*反馈数据区*/
u8 can2_board_buffer_first[18];
int16_t can2_201_208_buffer[16];

Encoder_t can2_encoder_201;
Encoder_t can2_encoder_202;
Encoder_t can2_encoder_203;
Encoder_t can2_encoder_204;
Encoder_t can2_encoder_205;
Encoder_t can2_encoder_206;
Encoder_t can2_encoder_207;
Encoder_t can2_encoder_208;
/*************************************************************************************************
*名称:	CAN2_RX0_IRQHandler
*功能:	can2 接收函数
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

        switch (RxMessage.StdId)
        {
            case 0x205:
            {
                can2_201_208_buffer[8] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
                can2_201_208_buffer[9] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
                CAN_DATA_Encoder_Deal(19, can2_201_208_buffer[8], can2_201_208_buffer[9], &can2_encoder_205);
                break;
            }
					
        case 0x401:
        {
            can2_board_buffer_first[0] = RxMessage.Data[0];
            can2_board_buffer_first[1] = RxMessage.Data[1];
            can2_board_buffer_first[2] = RxMessage.Data[2];
            can2_board_buffer_first[3] = RxMessage.Data[3];
            can2_board_buffer_first[4] = RxMessage.Data[4];
            can2_board_buffer_first[5] = RxMessage.Data[5];
            can2_board_buffer_first[6] = RxMessage.Data[6];
            can2_board_buffer_first[7] = RxMessage.Data[7];
            can2_board_buffer_first[16] = 1;
            break;
        }

        case 0x402:
        {
            can2_board_buffer_first[8] = RxMessage.Data[0];
            can2_board_buffer_first[9] = RxMessage.Data[1];
            can2_board_buffer_first[10] = RxMessage.Data[2];
            can2_board_buffer_first[11] = RxMessage.Data[3];
            can2_board_buffer_first[12] = RxMessage.Data[4];
            can2_board_buffer_first[13] = RxMessage.Data[5];
            can2_board_buffer_first[17] = 1;
            break;
        }

        /****1/13 增加夹取 **************/
        case 0x403:
        {
            if (RxMessage.Data[0] == 0x0A && RxMessage.Data[7] == 0xCE)
            {
                can2_board_buffer_first[0] = RxMessage.Data[1]; 
                can2_board_buffer_first[1] = RxMessage.Data[2]; 
                can2_board_buffer_first[2] = RxMessage.Data[3]; 
                can2_board_buffer_first[3] = RxMessage.Data[4]; 
                can2_board_buffer_first[4] = RxMessage.Data[5]; 
                can2_board_buffer_first[17] = 2;
            }
        }



        default:
            break;
        }
    }
}

/*************************************************************************************************
*名称:	CAN2_To_Board
*功能:	发送用户发送自定义数据
*形参: 	unsigned char*CAN_DATA
*返回:	无
*说明:	自己定义标识符
*************************************************************************************************/
void CAN2_To_Board(u8 *CAN_DATA, int16_t stdid)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg TxMessage; //定义一个发送信息的结构体

    TxMessage.StdId = stdid;      //根据820r设置标识符
    TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    TxMessage.DLC = 8;            //指定数据的长度
    TxMessage.Data[0] = (unsigned char)(CAN_DATA[0]);
    TxMessage.Data[1] = (unsigned char)(CAN_DATA[1]);
    TxMessage.Data[2] = (unsigned char)(CAN_DATA[2]);
    TxMessage.Data[3] = (unsigned char)(CAN_DATA[3]);
    TxMessage.Data[4] = (unsigned char)(CAN_DATA[4]);
    TxMessage.Data[5] = (unsigned char)(CAN_DATA[5]);
    TxMessage.Data[6] = (unsigned char)(CAN_DATA[6]);
    TxMessage.Data[7] = (unsigned char)(CAN_DATA[7]);
    mbox = CAN_Transmit(CAN2, &TxMessage); //发送信息
    i = 0;

    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}

/*************************************************************************************************
*名称:	CAN2_205_To_208_SEND
*功能:	can2 发送函数
*形参: 	int16_t control_205,int16_t control_206, int16_t control_207, int16_t control_208
*返回:	无
*说明:	无
*************************************************************************************************/
void CAN2_205_To_208_SEND(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208)
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

    mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;

    while((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))i++;	//等待发送结束
}

Encoder_t * Return_Can2_201_208_Encoder(u8 ch)
{
    switch(ch)
    {
        case 1:
        {
            return &can2_encoder_201;
        }

        case 2:
        {
            return &can2_encoder_202;
        }

        case 3:
        {
            return &can2_encoder_203;
        }

        case 4:
        {
            return &can2_encoder_204;
        }

        case 5:
        {
            return &can2_encoder_205;
        }

        case 6:
        {
            return &can2_encoder_206;
        }

        case 7:
        {
            return &can2_encoder_207;
        }

        case 8:
        {
            return &can2_encoder_208;
        }

        default:
            break;
    }

    return 0;
}
/*************************************************************************************************
*名称:	Return_CAN2_RC_DATA
*功能:	返回CAN2接收数据指针
*形参: 	无
*返回:	无
*说明:	返回夹取板子的状态值
*************************************************************************************************/
u8 *Return_CAN2_Board_Data(void)
{
    return can2_board_buffer_first;
}
