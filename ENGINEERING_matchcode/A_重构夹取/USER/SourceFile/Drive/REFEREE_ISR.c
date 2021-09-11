/*************************************************************************************************
ģ������ : ReferDeal
ʵ�ֹ��� : �������ϵͳ����
��    ע : ��
-------------------------------------------------------------------------------------------------
���� : 20-03-02
�汾 : 1.0
���� : ���ձ���״̬����
˵�� : Ŀǰֻ��ֻ������һ�����ݣ��ҷ������ݹ���δ���
			 �����Ჹ�룬����ӷ��͹���
			 �����Ի�ȡ����ָ���ȥд���룬����Ӱ��
-------------------------------------------------------------------------------------------------
���� : 20-04-09
�汾 : 1.1
���� : ���ӽ��ջ�����״̬��Ϣ������������Ϣ
˵�� : Ŀǰֻ��ֻ������һ������
			 ����ѡ�����жϽ������ݣ����ⲿ�������� -> ͨ��REFEREE.RECEIVE_FLAG��־λ
*************************************************************************************************/

#include "USART.h"
#include "REFEREE_ISR.h"

/*����ϵͳ���ݽṹ��*/
REFEREE_t REFEREE;
/*************************************************************************************************
*����:	Return_Referee_Point
*����:	��ò���ϵͳ���ݿ�����
*�β�: 	��
*����:	����ϵͳ���ݿ���ָ�� &REFEREE
*˵��:	��
*************************************************************************************************/
REFEREE_t*Return_Referee_Point(void)
{
    return &REFEREE;
}



/*����״̬*/
static void GAME_STATUS(REFEREE_t*referee, unsigned char k);
/*������״̬*/
static void ROBOT_STATUS(REFEREE_t*referee, unsigned char k);
/*��������*/
static void POWER_HEAT(REFEREE_t*referee, unsigned char k);

/*���ݶγ���*/
static const u8 HeaderLen = 5;
static const u8 CmdIdLen  = 2;
static const u8 CRC16Len  = 2;
/*����״̬���ݶγ���*/
static const u8 GameStatusLen  = (3 + CmdIdLen + CRC16Len);
/*������״̬���ݶγ���*/
static const u8 RobotStatusLen = (18 + CmdIdLen + CRC16Len);
/*�����������ݶγ���*/
static const u8 PowerHeatLen   = (16 + CmdIdLen + CRC16Len);


/*�ж�*/
void USART6_IRQHandler(void)
{
    unsigned short i = 0;
    unsigned short Len = 0;

    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)					  //�����жϱ�־λ
    {
        DMA_Cmd(DMA2_Stream1, DISABLE);															  //�ر�DMA,��ֹ�����ڼ�������

        while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);

        USART6->SR;
        USART6->DR;
        Len = 128 - DMA_GetCurrDataCounter(DMA2_Stream1);   					//DMA_GetCurrDataCounter���ڻ�ô�����ʣ��ĵ�Ԫ��
        REFEREE.DataLen = Len;

        for(i = 0; i < Len; i ++)
        {
            REFEREE.RefereeData[i] = referee_rx_buffer[i];
        }

        RefereeDataDeal(&REFEREE);
        DMA_SetCurrDataCounter(DMA2_Stream1, 128);           					//����Ҫ��������ݵ�Ԫ��
        DMA_Cmd(DMA2_Stream1, ENABLE);																//����DMA
        REFEREE.RECEIVE_FLAG = 1;
    }
}



/*�������ݽ������ݴ���*/
void RefereeDataDeal(REFEREE_t*referee)
{
    u8 i, k;

    for(i = 0; i < referee->DataLen; i ++)
    {
        if(referee->RefereeData[i] == 0xA5)																//֡ͷ
        {
            if(Verify_CRC8_Check_Sum(referee->RefereeData, HeaderLen) == 1)	//CRC8У��
            {
                referee->RealLen = ( (referee->RefereeData[i + 1]) | (referee->RefereeData[i + 2] << 8) );											//���ݳ���
                referee->Cmd_ID = ( (referee->RefereeData[i + HeaderLen]) | (referee->RefereeData[i + HeaderLen + 1] << 8) );   //������ID

                for(k = 0; k < 7; k ++)
                    referee->RealData[k] = referee->RefereeData[k + i];             //��ȡ����

                if(referee->Cmd_ID == 0x0001)
                {
                    GAME_STATUS(referee, (i + HeaderLen + CmdIdLen));								//����״̬���ݴ���
                    i = i + HeaderLen + GameStatusLen - 1;
                }

                else if(referee->Cmd_ID == 0x0003)																//������Ѫ��
                {

                }
                else if(referee->Cmd_ID == 0x0004)																//���ڷ���״̬
                {

                }
                else if(referee->Cmd_ID == 0x0101)																//�����¼�����
                {

                }
                else if(referee->Cmd_ID == 0x0102)																//����վ������ʶ����
                {

                }
                else if(referee->Cmd_ID == 0x0105)																//���ڷ��������
                {

                }
                else if(referee->Cmd_ID == 0x0201)																//������״̬����
                {
                    ROBOT_STATUS(referee, (i + HeaderLen + CmdIdLen));
                    i = i + HeaderLen + RobotStatusLen - 1;
                }
                else if(referee->Cmd_ID == 0x0202)																//������������
                {
                    POWER_HEAT(referee, (i + HeaderLen + CmdIdLen));
                    i = i + HeaderLen + PowerHeatLen - 1;
                }
                else if(referee->Cmd_ID == 0x0204)																//��������������
                {

                }
                else if(referee->Cmd_ID == 0x0205)																//�ɻ�����
                {

                }
                else if(referee->Cmd_ID == 0x0206)																//�˺�״̬
                {

                }
                else if(referee->Cmd_ID == 0x0207)																//ʵʱ�����Ϣ
                {

                }
                else if(referee->Cmd_ID == 0x0208)																//�ӵ�ʣ�෢����
                {

                }
                else if(referee->Cmd_ID == 0x0209)																//RFID״̬
                {

                }
            }
        }
    }

    REFEREE.RECEIVE_FLAG = 0;					//������һ������
}



/*����״̬����*/
static void GAME_STATUS(REFEREE_t*referee, unsigned char k)
{
    u8 j;

    for(j = 0; j < GameStatusLen; j ++)			//��������
        referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];

    if(Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen + GameStatusLen) ) == 1) //CRC16У��
    {
        referee->GameStatus.game_type = referee->RealData[7];
        referee->GameStatus.game_progress = (referee->RealData[7] >> 4);
        referee->GameStatus.stage_remain_time = ( (referee->RealData[8]) | (referee->RealData[9] << 8) );
        referee->GameStatus.error = 0;
    }
    else
    {
        referee->GameStatus.error = 1;
    }
}


/*������״̬����*/
static void ROBOT_STATUS(REFEREE_t*referee, unsigned char k)
{
    u8 j;

    for(j = 0; j < RobotStatusLen; j ++)			//��������
        referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];

    if(Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen + RobotStatusLen) ) == 1) //CRC16У��
    {
        referee->RobotStatus.robot_id = referee->RealData[7];
        referee->RobotStatus.robot_level = referee->RealData[8];
        referee->RobotStatus.remain_HP = ( (referee->RealData[9]) | (referee->RealData[10] << 8) );
        referee->RobotStatus.max_HP = ( (referee->RealData[11]) | (referee->RealData[12] << 8) );
        referee->RobotStatus.shooter_heat0_cooling_rate = ( (referee->RealData[13]) | (referee->RealData[14] << 8) );
        referee->RobotStatus.shooter_heat0_cooling_limit = ( (referee->RealData[15]) | (referee->RealData[16] << 8) );
        referee->RobotStatus.shooter_heat1_cooling_rate = ( (referee->RealData[17]) | (referee->RealData[18] << 8) );
        referee->RobotStatus.shooter_heat1_cooling_limit = ( (referee->RealData[19]) | (referee->RealData[20] << 8) );
        referee->RobotStatus.shooter_heat0_speed_limit = (referee->RealData[21]);
        referee->RobotStatus.shooter_heat1_speed_limit = (referee->RealData[22]);
        referee->RobotStatus.max_chassis_power = (referee->RealData[23]);
        referee->RobotStatus.mains_power_gimbal_output = referee->RealData[24];
        referee->RobotStatus.mains_power_chassis_output = (referee->RealData[24] >> 1);
        referee->RobotStatus.mains_power_shooter_output = (referee->RealData[24] >> 2);

        referee->RobotStatus.error = 0;
    }
    else
    {
        referee->RobotStatus.error = 1;
    }

}

static void POWER_HEAT(REFEREE_t*referee, unsigned char k)
{
    u8 j;
    Float_t F;

    for(j = 0; j < PowerHeatLen; j ++)			//��������
        referee->RealData[HeaderLen + CmdIdLen + j] = referee->RefereeData[k + j];

    if(Verify_CRC16_Check_Sum(referee->RealData, (HeaderLen + PowerHeatLen) ) == 1) //CRC16У��
    {
        referee->PowerHeat.chassis_volt = ( (referee->RealData[7]) | (referee->RealData[8] << 8) );
        referee->PowerHeat.chassis_current = ( (referee->RealData[9]) | (referee->RealData[10] << 8) );
        F.float_byte.LB = (referee->RealData[11]);
        F.float_byte.MLB = (referee->RealData[12]);
        F.float_byte.MHB = (referee->RealData[13]);
        F.float_byte.HB = (referee->RealData[14]);
        referee->PowerHeat.chassis_power = F.value;
        referee->PowerHeat.chassis_power_buffer = ( (referee->RealData[15]) | (referee->RealData[16] << 8) );
        referee->PowerHeat.shooter_heat0 = ( (referee->RealData[17]) | (referee->RealData[18] << 8) );
        referee->PowerHeat.shooter_heat1 = ( (referee->RealData[19]) | (referee->RealData[20] << 8) );
        referee->PowerHeat.mobile_shooter_heat2 = ( (referee->RealData[21]) | (referee->RealData[22] << 8) );
    }
    else
    {
        referee->PowerHeat.error = 1;
    }
}
