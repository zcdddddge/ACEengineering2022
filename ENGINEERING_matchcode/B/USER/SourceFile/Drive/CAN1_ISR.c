#include "CAN1_ISR.h"

/*���ݽ�����*/
int16_t can1_201_204_buffer[8];
int16_t can1_205_208_buffer[8];

/*����ֵ�ṹ��*/
Encoder_t can1_encoder_201;	
Encoder_t can1_encoder_202;	
Encoder_t can1_encoder_203;	
Encoder_t can1_encoder_204;	
Encoder_t can1_encoder_205;	
Encoder_t can1_encoder_206;	
Encoder_t can1_encoder_207;	
Encoder_t can1_encoder_208;	



/*************************************************************************************************
*����:	CAN1_RX0_IRQHandler
*����:	can1 ���պ���
*�β�: 	��
*����:	��
*˵��:	��
*************************************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		
		switch(RxMessage.StdId)
		{
			case 0x201:
			{
				can1_201_204_buffer[0] = (RxMessage.Data[0]<<8)+RxMessage.Data[1];    
				can1_201_204_buffer[1] = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				CAN_DATA_Speed_Deal(can1_201_204_buffer[1],&can1_encoder_201);
				break;
			}
			case 0x202:
			{
				can1_201_204_buffer[2] = (RxMessage.Data[0]<<8)+RxMessage.Data[1];
				can1_201_204_buffer[3] = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				CAN_DATA_Speed_Deal(can1_201_204_buffer[3],&can1_encoder_202);
				break;
			}
			case 0x203:
			{
				can1_201_204_buffer[4] = (RxMessage.Data[0]<<8)+RxMessage.Data[1]; 
				can1_201_204_buffer[5] = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				CAN_DATA_Speed_Deal(can1_201_204_buffer[5],&can1_encoder_203);
				break;
			}			
			case 0x204:
			{
				can1_201_204_buffer[6] = (RxMessage.Data[0]<<8)+RxMessage.Data[1];  
				can1_201_204_buffer[7] = (RxMessage.Data[2]<<8)+RxMessage.Data[3];
				CAN_DATA_Speed_Deal(can1_201_204_buffer[7],&can1_encoder_204);
				break;
			}
			case 0x205:
			{
				can1_205_208_buffer[0] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
				can1_205_208_buffer[1] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
				CAN_DATA_Encoder_Deal(36,can1_205_208_buffer[0],can1_205_208_buffer[1],&can1_encoder_205);
				break;	
			}	
			case 0x206:        
			{
				can1_205_208_buffer[2] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
				can1_205_208_buffer[3] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
				CAN_DATA_Encoder_Deal(36,can1_205_208_buffer[2],can1_205_208_buffer[3],&can1_encoder_206);

				break;
			}	
			case 0x20B:
			{
				can1_205_208_buffer[4] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
				can1_205_208_buffer[5] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
				CAN_DATA_Encoder_Deal(1,can1_205_208_buffer[4],can1_205_208_buffer[5],&can1_encoder_207);

				break;	
			}	
			case 0x208:        
			{
				can1_205_208_buffer[6] = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
				can1_205_208_buffer[7] = (RxMessage.Data[2] << 8) | RxMessage.Data[3];
				CAN_DATA_Encoder_Deal(36,can1_205_208_buffer[6],can1_205_208_buffer[7],&can1_encoder_208);

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
*����:	CAN1_201_To_204_SEND
*����:	can1 ���ͺ���
*�β�: 	int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204
*����:	��
*˵��:	��
*************************************************************************************************/
void CAN1_201_To_204_SEND(int16_t ESC_201,int16_t ESC_202,int16_t ESC_203,int16_t ESC_204)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;     									//����һ��������Ϣ�Ľṹ��
	
	TxMessage.StdId = 0x200;      						//����820r���ñ�ʶ��
	TxMessage.IDE = CAN_ID_STD;   						//ָ����Ҫ�������Ϣ�ı�ʶ��������
	TxMessage.RTR = CAN_RTR_DATA; 						//ָ����֡�����������Ϣ������   ����֡��Զ��֡
	TxMessage.DLC = 8;      									//ָ�����ݵĳ���
	TxMessage.Data[0] = (unsigned char)(ESC_201>>8);
	TxMessage.Data[1] = (unsigned char)(ESC_201);
	TxMessage.Data[2] = (unsigned char)(ESC_202>>8);
	TxMessage.Data[3] = (unsigned char)(ESC_202);
	TxMessage.Data[4] = (unsigned char)(ESC_203>>8);
	TxMessage.Data[5] = (unsigned char)(ESC_203);
	TxMessage.Data[6] = (unsigned char)(ESC_204>>8);
	TxMessage.Data[7] = (unsigned char)(ESC_204);
	mbox=CAN_Transmit(CAN1,&TxMessage);  			//������Ϣ
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))			
		i++;   
}




/*************************************************************************************************
*����:	CAN1_205_To_208_SEND
*����:	can1 ���ͺ���
*�β�: 	int16_t control_205,int16_t control_206, int16_t control_207, int16_t control_208
*����:	��
*˵��:	��
*************************************************************************************************/
void CAN1_205_To_208_SEND(int16_t ESC_205,int16_t ESC_206,int16_t ESC_207, int16_t ESC_208)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x1ff;	 // ��׼��ʶ��Ϊ00x1f
	TxMessage.IDE=0;		     // ʹ����չ��ʶ��
	TxMessage.RTR=0;		     // ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.DLC=8;		     // ������֡��Ϣ
	TxMessage.Data[0] = ESC_205>>8;
	TxMessage.Data[1] = ESC_205;
	TxMessage.Data[2] = ESC_206>>8;
	TxMessage.Data[3] = ESC_206;
	TxMessage.Data[4] = ESC_207>>8;
	TxMessage.Data[5] = ESC_207;
	TxMessage.Data[6] = ESC_208>>8;
	TxMessage.Data[7] = ESC_208;
      
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
}

void CAN1_205_To_206_SEND(int16_t ESC_205,int16_t ESC_206)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x1ff;	 // ��׼��ʶ��Ϊ00x1f
	TxMessage.IDE=0;		     // ʹ����չ��ʶ��
	TxMessage.RTR=0;		     // ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.DLC=4;		     // ������֡��Ϣ
	TxMessage.Data[0] = ESC_205>>8;
	TxMessage.Data[1] = ESC_205;
	TxMessage.Data[2] = ESC_206>>8;
	TxMessage.Data[3] = ESC_206;
      
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
}

void CAN1_207_To_208_SEND(int16_t ESC_207, int16_t ESC_208)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x1ff;	 // ��׼��ʶ��Ϊ00x1f
	TxMessage.IDE=0;		     // ʹ����չ��ʶ��
	TxMessage.RTR=0;		     // ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.DLC=4;		     // ������֡��Ϣ

	TxMessage.Data[4] = ESC_207>>8;
	TxMessage.Data[5] = ESC_207;
	TxMessage.Data[6] = ESC_208>>8;
	TxMessage.Data[7] = ESC_208;
      
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
}



/*************************************************************************************************
*����:	CAN1_205_To_208_SEND
*����:	can1 ���ͺ���
*�β�: 	int16_t control_205,int16_t control_206, int16_t control_207, int16_t control_208
*����:	��
*˵��:	��
*************************************************************************************************/
void CAN1_SEND_6020_7(int16_t ESC_207)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x2FF;	 // ��׼��ʶ��Ϊ0x2ff
	TxMessage.IDE=0;		     // ʹ����չ��ʶ��
	TxMessage.RTR=0;		     // ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.DLC=8;		     // ������֡��Ϣ
	TxMessage.Data[0] = 0>>8;
	TxMessage.Data[1] = 0;
	TxMessage.Data[2] = 0>>8;
	TxMessage.Data[3] = 0;
	TxMessage.Data[4] = ESC_207>>8;
	TxMessage.Data[5] = ESC_207;
	TxMessage.Data[6] = 0>>8;
	TxMessage.Data[7] = 0;
      
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
}




/*************************************************************************************************
*����:	CAN1_To_Board
*����:	�����û������Զ�������
*�β�: 	unsigned char*CAN_DATA , int16_t stdid
*����:	��
*˵��:	�Լ������ʶ��
*************************************************************************************************/
void CAN1_To_Board(u8*CAN_DATA,int16_t stdid)
{
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;     									//����һ��������Ϣ�Ľṹ��
	
	TxMessage.StdId = stdid;      						//����820r���ñ�ʶ��
	TxMessage.IDE = CAN_ID_STD;   						//ָ����Ҫ�������Ϣ�ı�ʶ��������
	TxMessage.RTR = CAN_RTR_DATA; 						//ָ����֡�����������Ϣ������   ����֡��Զ��֡
	TxMessage.DLC = 8;      									//ָ�����ݵĳ���
	TxMessage.Data[0] = (unsigned char)(CAN_DATA[0]);
	TxMessage.Data[1] = (unsigned char)(CAN_DATA[1]);
	TxMessage.Data[2] = (unsigned char)(CAN_DATA[2]);
	TxMessage.Data[3] = (unsigned char)(CAN_DATA[3]);
	TxMessage.Data[4] = (unsigned char)(CAN_DATA[4]);
	TxMessage.Data[5] = (unsigned char)(CAN_DATA[5]);
	TxMessage.Data[6] = (unsigned char)(CAN_DATA[6]);
	TxMessage.Data[7] = (unsigned char)(CAN_DATA[7]);
	mbox=CAN_Transmit(CAN1,&TxMessage);  			//������Ϣ
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))			
		i++;
}



/*************************************************************************************************
*����:	Return_Can1_205_Encoder
*����:	����Can1_205���ݴ���ָ��
*�β�: 	��
*����:	�ṹ���ַ
*˵��:	��
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
		case 6 :
		{
			return &can1_encoder_206;
		}
		case 7 : 
		{
			return &can1_encoder_207;
 		}
		case 8 :
		{
			return &can1_encoder_208;
		}
		default:
			break;
	}
		return 0;
}


