#include "CAN2_ISR.h"



/*反馈数据区*/
uint8_t can2_board_buffer[8];

int16_t can2_205_208_buffer[8];

/*码盘值结构体*/
Encoder_t can2_encoder_205;
Encoder_t can2_encoder_206;
Encoder_t can2_encoder_207;	

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
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET){	
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
		CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
		if(RxMessage.StdId ==0x11) 
		{
				can2_board_buffer[0]= RxMessage.Data[0] ;  //ch0 
				can2_board_buffer[1]= RxMessage.Data[1] ;
				can2_board_buffer[2]= RxMessage.Data[2] ;
				can2_board_buffer[3]= RxMessage.Data[3] ;
				can2_board_buffer[4]= RxMessage.Data[4] ;
				can2_board_buffer[5]= RxMessage.Data[5] ;
				can2_board_buffer[6]= RxMessage.Data[6] ;
				can2_board_buffer[7]= RxMessage.Data[7] ;
		}
		else if(RxMessage.StdId ==0x206)
		{
				can2_205_208_buffer[2] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
				can2_205_208_buffer[3] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
				CAN_DATA_Encoder_Deal(36,can2_205_208_buffer[2],can2_205_208_buffer[3],&can2_encoder_206);
		}
		
			
	}			
	
}




/*************************************************************************************************
*名称:	CAN2_201_To_204_SEND
*功能:	can2 发送函数
*形参: 	int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204
*返回:	无
*说明:	无
*************************************************************************************************/
void CAN2_201_To_204_SEND(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;     																									//定义一个发送信息的结构体
	
	TxMessage.StdId = 0x200;      																						//根据820r设置标识符
	TxMessage.IDE = CAN_ID_STD;   																						//指定将要传输的消息的标识符的类型
	TxMessage.RTR = CAN_RTR_DATA; 																						//指定的帧将被传输的消息的类型   数据帧或远程帧
	TxMessage.DLC = 8;      																									//指定数据的长度
	TxMessage.Data[0] = (unsigned char)(ESC_201>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_201);
	TxMessage.Data[2] = (unsigned char)(ESC_202>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_202);
	TxMessage.Data[4] = (unsigned char)(ESC_203>>8);
	TxMessage.Data[5] = (unsigned char)(ESC_203);
	TxMessage.Data[6] = (unsigned char)(ESC_204>>8);
	TxMessage.Data[7] = (unsigned char)(ESC_204);
	mbox=CAN_Transmit(CAN2,&TxMessage);   																			//发送信息
	i=0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))			//等待发送结束
		i++;   
}




/*************************************************************************************************
*名称:	CAN2_205_To_208_SEND
*功能:	can2 发送函数
*形参: 	int16_t ESC_205,int16_t ESC_206,int16_t ESC_207,int16_t ESC_208
*返回:	无
*说明:	无
*************************************************************************************************/
void CAN2_205_To_208_SEND(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207,int16_t ESC_208)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x1ff;	 // 标准标识符为0
	TxMessage.IDE=0;		     // 使用扩展标识符
	TxMessage.RTR=0;		     // 消息类型为数据帧，一帧8位
	TxMessage.DLC=8;		     // 发送两帧信息
	TxMessage.Data[0] = ESC_205>>8;
	TxMessage.Data[1] = ESC_205;
	TxMessage.Data[2] = ESC_206>>8;
	TxMessage.Data[3] = ESC_206;
	TxMessage.Data[4] = ESC_207>>8;
	TxMessage.Data[5] = ESC_207;
	TxMessage.Data[6] = ESC_208>>8;
	TxMessage.Data[7] = ESC_208;
      
	mbox= CAN_Transmit(CAN2, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
}



/*************************************************************************************************
*名称:	CAN2_To_Board
*功能:	发送用户发送自定义数据
*形参: 	unsigned char*CAN_DATA
*返回:	无
*说明:	自己定义标识符
*************************************************************************************************/
void CAN2_To_Board(u8*CAN_DATA,int16_t stdid)
{
	u8 mbox;
	u16 i=0;
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
	mbox=CAN_Transmit(CAN2,&TxMessage);  			//发送信息
	i=0;
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))			
		i++;
}



/*************************************************************************************************
*名称:	Return_CAN2_RC_DATA
*功能:	返回CAN2接收数据指针
*形参: 	无
*返回:	无
*说明:	返回夹取板子的状态值
*************************************************************************************************/
uint8_t  *Return_CAN2_Board_Data(void)
{
		return can2_board_buffer;
}



/*************************************************************************************************
*名称:	Return_Can2_205_Encoder
*功能:	返回Can2_205数据处理指针
*形参: 	无
*返回:	结构体地址
*说明:	无
*************************************************************************************************/
Encoder_t * Return_Can2_205_Encoder(void)
{
	return &can2_encoder_205;
}



/*************************************************************************************************
*名称:	Return_Can2_206_Encoder
*功能:	返回Can2_206数据处理指针
*形参: 	无
*返回:	结构体地址
*说明:	无
*************************************************************************************************/
Encoder_t * Return_Can2_206_Encoder(void)
{
	return &can2_encoder_206;
}



/*************************************************************************************************
*名称:	Return_Can2_207_Encoder
*功能:	返回Can2_207数据处理指针
*形参: 	无
*返回:	结构体地址
*说明:	无
*************************************************************************************************/
Encoder_t * Return_Can2_207_Encoder(void)
{
	return &can2_encoder_207;
}

Encoder_t * Return_Can2_201_208_Encoder(u8 ch)
{
		switch(ch)
	{

		case 6 :
		{
			return &can2_encoder_206;
		}

		default:
			break;
	}
		return 0;
}
