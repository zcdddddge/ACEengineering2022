#include "CAN.h"

/*************************************************************************************************
*����:	CAN1_Init_GPIOA_IO11_IO12
*����:	CAN1��ʼ������
*�β�: 	mode->CAN_Mode_Normal CAN_Mode_LoopBack CAN_Mode_Silent CAN_Mode_Silent_LoopBack
*����:	��
*˵��:	CAN�ǹ�����APB1ʱ��(0.25����ʱ��)
				CAN BaudRate = APB1/(tsjw+tbs2+tbs1)/brp
*************************************************************************************************/
void CAN1_Init_GPIOA_IO11_IO12(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
		NVIC_InitTypeDef       NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);					//��GPIOʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);			//�������Ÿ��ù���
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;  		//CAN1_TX(PD1) CAN1_RX(PD0)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									//�����������
		GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;				
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    
    CAN_DeInit(CAN1);                        						//����Χ�Ĵ�����ʼ�������ǵ�Ĭ������ֵ
    CAN_StructInit(&CAN_InitStructure);      						//������Ĭ��ֵ���ÿ��CAN_InitStructure��Ա
    /************CAN���ߵ�����*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE;								//��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM = DISABLE;								//����Զ����߹���ģʽ
    CAN_InitStructure.CAN_AWUM = DISABLE;								//�Զ�����ģʽ��˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART = DISABLE;								//���Զ��ش���ģʽ����ֹ�����Զ����� 
    CAN_InitStructure.CAN_RFLM = DISABLE;								//����FIFO����ģʽ�����Ĳ�����,�µĸ��Ǿɵ� 
    CAN_InitStructure.CAN_TXFP = ENABLE;								//����FIFO���ȼ������ȼ��ɱ��ı�ʶ������ 
    CAN_InitStructure.CAN_Mode = mode;					
		CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;   
    CAN_Init(CAN1, &CAN_InitStructure);
		/******CAN���ߵĹ�������(��������)********/
    CAN_FilterInitStructure.CAN_FilterNumber=0;     						//������ 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;					//the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);										//�˲�����ʼ��   

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);												//�����ж�
}



/*************************************************************************************************
*����:	CAN1_Init_GPIOD_IO0_IO1
*����:	CAN1��ʼ������
*�β�: 	mode->CAN_Mode_Normal CAN_Mode_LoopBack CAN_Mode_Silent CAN_Mode_Silent_LoopBack
*����:	��
*˵��:	CAN�ǹ�����APB1ʱ��(0.25����ʱ��)
				CAN BaudRate = APB1/(tsjw+tbs2+tbs1)/brp
*************************************************************************************************/
void CAN1_Init_GPIOD_IO0_IO1(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
		NVIC_InitTypeDef       NVIC_InitStructure;

	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);		 //��GPIOʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);     //��can1ʱ��
		
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);	 //�������Ÿ��ù���
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);  //�������Ÿ��ù���

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						
		GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;				
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;   	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    
    CAN_DeInit(CAN1);                        						//����Χ�Ĵ�����ʼ�������ǵ�Ĭ������ֵ
    CAN_StructInit(&CAN_InitStructure);      						//������Ĭ��ֵ���ÿ��CAN_InitStructure��Ա
    /************CAN���ߵ�����*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE;								//��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM = ENABLE;								//����Զ����߹���ģʽ
    CAN_InitStructure.CAN_AWUM = DISABLE;								//�Զ�����ģʽ��˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART = DISABLE;								//���Զ��ش���ģʽ����ֹ�����Զ����� 
    CAN_InitStructure.CAN_RFLM = DISABLE;								//����FIFO����ģʽ�����Ĳ�����,�µĸ��Ǿɵ� 
    CAN_InitStructure.CAN_TXFP = ENABLE;								//����FIFO���ȼ������ȼ��ɱ��ı�ʶ������ 
    CAN_InitStructure.CAN_Mode = mode;					
		CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;   
    CAN_Init(CAN1, &CAN_InitStructure);
		/******CAN���ߵĹ�������(��������)********/
    CAN_FilterInitStructure.CAN_FilterNumber=0;     						//������ 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;					//the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);										//�˲�����ʼ��   

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);												//�����ж�
}


/*************************************************************************************************
*����:	CAN2_Init_GPIOB_IO12_IO13
*����:	CAN2��ʼ������
*�β�: 	mode->CAN_Mode_Normal CAN_Mode_LoopBack CAN_Mode_Silent CAN_Mode_Silent_LoopBack
*����:	��
*˵��:	CAN�ǹ�����APB1ʱ��(0.25����ʱ��)
				CAN BaudRate = APB1/(tsjw+tbs2+tbs1)/brp
*************************************************************************************************/
void CAN2_Init_GPIOB_IO12_IO13(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
		NVIC_InitTypeDef       NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);					//��GPIOʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);	

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);			//�������Ÿ��ù���
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;									//�����������
    GPIO_InitStructure.GPIO_OType=  GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;					
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure); 
    
    CAN_DeInit(CAN2);                        								//����Χ�Ĵ�����ʼ�������ǵ�Ĭ������ֵ
    CAN_StructInit(&CAN_InitStructure);      								//������Ĭ��ֵ���ÿ��CAN_InitStructure��Ա
    /************CAN���ߵ�����*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE;										//��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_ABOM = DISABLE;										//����Զ����߹���ģʽ
    CAN_InitStructure.CAN_AWUM = DISABLE;										//�Զ�����ģʽ��˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_InitStructure.CAN_NART = DISABLE;										//���Զ��ش���ģʽ����ֹ�����Զ����� 
    CAN_InitStructure.CAN_RFLM = DISABLE;										//����FIFO����ģʽ�����Ĳ�����,�µĸ��Ǿɵ� 
    CAN_InitStructure.CAN_TXFP = ENABLE;										//����FIFO���ȼ������ȼ��ɱ��ı�ʶ������ 
    CAN_InitStructure.CAN_Mode = mode;											//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ;
    CAN_InitStructure.CAN_SJW  = tsjw;
    CAN_InitStructure.CAN_BS1=tbs1; 												//Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
		CAN_InitStructure.CAN_BS2=tbs2;						  						//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = brp;   							  //brp    
    CAN_Init(CAN2, &CAN_InitStructure);
		/******CAN���ߵĹ�������(��������)********/
    CAN_FilterInitStructure.CAN_FilterNumber=14;                  	  	 //������ 0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;    //the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);							  							//�˲�����ʼ��   

	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);							  	  							//�����ж� 
}



/*************************************************************************************************
*����:	CAN2_Init_GPIOB_IO5_IO6
*����:	CAN2��ʼ������
*�β�: 	mode->CAN_Mode_Normal CAN_Mode_LoopBack CAN_Mode_Silent CAN_Mode_Silent_LoopBack
*����:	��
*˵��:	CAN�ǹ�����APB1ʱ��(0.25����ʱ��)
				CAN BaudRate = APB1/(tsjw+tbs2+tbs1)/brp
*************************************************************************************************/
void CAN2_Init_GPIOB_IO5_IO6(uint8_t tsjw, uint8_t tbs1, uint8_t tbs2, uint16_t brp, uint8_t mode)
{
			CAN_InitTypeDef        CAN_Init2;
    CAN_FilterInitTypeDef  CAN_Filter2;
    GPIO_InitTypeDef       GPIO_Init_can2;
		NVIC_InitTypeDef       NVIC_Init_can2;                  //����ṹ��

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		//��GPIOʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);    //��can1ʱ��

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);	//�������Ÿ��ù���
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);  //�������Ÿ��ù���

    GPIO_Init_can2.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init_can2.GPIO_Mode = GPIO_Mode_AF;								  //�����������
    GPIO_Init_can2.GPIO_OType=  GPIO_OType_PP;
		GPIO_Init_can2.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init_can2.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_Init_can2);
    //CAN����io������ 
		NVIC_Init_can2.NVIC_IRQChannel = CAN2_RX0_IRQn;					//�ж����� 
		NVIC_Init_can2.NVIC_IRQChannelPreemptionPriority = 2;           //��ռ���ȼ� 1
		NVIC_Init_can2.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_Init_can2);

    CAN_DeInit(CAN2);                        //����Χ�Ĵ�����ʼ�������ǵ�Ĭ������ֵ
    CAN_StructInit(&CAN_Init2);      //������Ĭ��ֵ���ÿ��CAN_InitStructure��Ա
    //CAN���ߵ�����
    CAN_Init2.CAN_TTCM = DISABLE;			      //��ʱ�䴥��ͨ��ģʽ
    CAN_Init2.CAN_ABOM = DISABLE;			      //����Զ����߹���ģʽ
    CAN_Init2.CAN_AWUM = DISABLE;			      //�Զ�����ģʽ��˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
    CAN_Init2.CAN_NART = DISABLE;			      //���Զ��ش���ģʽ����ֹ�����Զ����� 
    CAN_Init2.CAN_RFLM = DISABLE;			      //����FIFO����ģʽ�����Ĳ�����,�µĸ��Ǿɵ� 
    CAN_Init2.CAN_TXFP = ENABLE;			      //����FIFO���ȼ������ȼ��ɱ��ı�ʶ������ 
    CAN_Init2.CAN_Mode = mode;						//ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ;
    CAN_Init2.CAN_SJW  = tsjw;
    CAN_Init2.CAN_BS1 = tbs1;
    CAN_Init2.CAN_BS2 = tbs2;
    CAN_Init2.CAN_Prescaler = brp;   //CAN BaudRate 42/(1+5+8)/3=1Mbps  (�����can������Ϊ1m)
    CAN_Init(CAN2, &CAN_Init2);
		//CAN���ߵĹ�������(��������)
    CAN_Filter2.CAN_FilterNumber=14;                  //������ 0
    CAN_Filter2.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_Filter2.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_Filter2.CAN_FilterIdHigh=0x0000;
    CAN_Filter2.CAN_FilterIdLow=0x0000;
    CAN_Filter2.CAN_FilterMaskIdHigh=0x0000;
    CAN_Filter2.CAN_FilterMaskIdLow=0x0000;
    CAN_Filter2.CAN_FilterFIFOAssignment=0;
    CAN_Filter2.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN_Filter2);			//�˲�����ʼ��   

		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);		//�����ж�    ���û����ָ����CANx�ж�    CAN_IT_FMP0:�ȴ��жϵ�FIFO 0��Ϣ	
}
