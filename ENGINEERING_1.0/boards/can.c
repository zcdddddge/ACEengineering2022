#include "can.h"

/* can1 */
uint8_t can1_mode_init(uint8_t SJW, uint8_t BS1, uint8_t BS2, uint8_t Prescaler, uint8_t mode)
{
    //CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    //  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, ENABLE);
    //  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, DISABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100Hz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /************����*******************/
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    /************CAN���ߵ�����*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE; //ʱ�䴥��ͨѶ��1:ʹ��,0:��ֹ
    CAN_InitStructure.CAN_ABOM = ENABLE;  //1:������������߹ر�,Ӳ���Զ��ָ�,0:������ָ�
    CAN_InitStructure.CAN_AWUM = ENABLE;  //1:�Զ��˳�˯��ģʽ,0:������˳�˯��ģʽ
    CAN_InitStructure.CAN_NART = DISABLE; //1:���ܳɰ�ֻ����һ��,0:����ʧ�ܻ��ط�
    CAN_InitStructure.CAN_RFLM = DISABLE; //1:�����������Ϣ,0:���Ǿ���Ϣ����������Ϣ
    CAN_InitStructure.CAN_TXFP = DISABLE; //1:����������˳�����ȷ���,0:����ʶ�����ȷ���
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW = SJW;
    CAN_InitStructure.CAN_BS1 = BS1;             //Tbs1��ΧCAN_BS1_1tq ~ CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2 = BS2;             //Tbs2��ΧCAN_BS2_1tq ~ CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = Prescaler; //brp    CAN BaudRate 45/(1+8+6)/3=1Mbps
    CAN_Init(CAN1, &CAN_InitStructure);

    /******CAN���ߵĹ�������(��������)********/
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;       //�ж�����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //��Ӧ���ȼ�  0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return 0;
}

/* can2 */
uint8_t can2_mode_init(uint8_t SJW, uint8_t BS1, uint8_t BS2, uint8_t Prescaler, uint8_t mode)
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //��GPIOʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2); //�������Ÿ��ù���
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //�����������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;       //�ж�����
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //��Ӧ���ȼ� 0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_DeInit(CAN2);                   //����Χ�Ĵ�����ʼ�������ǵ�Ĭ������ֵ
    CAN_StructInit(&CAN_InitStructure); //������Ĭ��ֵ���ÿ��CAN_InitStructure��Ա

    /************CAN���ߵ�����*******************/
    CAN_InitStructure.CAN_TTCM = DISABLE; //ʱ�䴥��ͨѶ��1:ʹ��,0:��ֹ
    CAN_InitStructure.CAN_ABOM = ENABLE;  //1:������������߹ر�,Ӳ���Զ��ָ�,0:������ָ�
    CAN_InitStructure.CAN_AWUM = ENABLE;  //1:�Զ��˳�˯��ģʽ,0:������˳�˯��ģʽ
    CAN_InitStructure.CAN_NART = DISABLE; //1:���ܳɰ�ֻ����һ��,0:����ʧ�ܻ��ط�
    CAN_InitStructure.CAN_RFLM = DISABLE; //1:�����������Ϣ,0:���Ǿ���Ϣ����������Ϣ
    CAN_InitStructure.CAN_TXFP = ENABLE;  //1:����������˳�����ȷ���,0:����ʶ�����ȷ���
    CAN_InitStructure.CAN_Mode = mode;    //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ;
    CAN_InitStructure.CAN_SJW = SJW;
    CAN_InitStructure.CAN_BS1 = BS1;             //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2 = BS2;             //Tbs2��ΧCAN_BS2_1tq ~    CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = Prescaler; //brp    CAN BaudRate 45/(1+8+6)/3=1Mbps
    CAN_Init(CAN2, &CAN_InitStructure);

    /******CAN���ߵĹ�������(��������)********/
    CAN_FilterInitStructure.CAN_FilterNumber = 14; //������ 0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //the message which pass the filter save in fifo0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure); //�˲�����ʼ��

    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE); //�����ж�    ���û����ָ����CANx�ж�    CAN_IT_FMP0:�ȴ��жϵ�FIFO 0��Ϣ

    return 0;
}
