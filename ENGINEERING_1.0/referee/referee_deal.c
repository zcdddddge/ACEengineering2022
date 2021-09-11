#include "referee_deal.h"
#include "usart6.h"
#include "uart4.h"
#include "crc.h"
#include "parameter.h"
#include "fifo_buff.h"

//static
REFEREE_t Referee;          //����ϵͳ���ݽṹ��
xFrameHeader_t FrameHeader; //����֡ͷ��Ϣ

extern fifo_rx_def fifo_uart_rx_4;
fifo_rx_def *pfifo_referee = &fifo_uart_rx_4;

/**
  * @brief          ��������ʼ��
  * @param[in]      none
  * @retval         none
  * @attention
  */
void referee_system_init(void)
{
    uart4_init(5); //����ϵͳ
}

/**
  * @brief          ��ò���ϵͳ���ݿ�����
  * @param[in]      none
  * @retval         ����ϵͳ���ݿ���ָ�� &Referee
  * @attention
  */
REFEREE_t *return_referee_point(void)
{
    return &Referee;
}

/*���ݶγ���*/
static const uint8_t HeaderLen = 5;
static const uint8_t CmdIdLen = 2;
static const uint8_t CRC16Len = 2;

bool_t Judge_Data_TF = 0; //���������Ƿ����,������������

/**
  * @brief          ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
  * @param[in]      ���ղ���ϵͳ���ݽṹ��,��������
  * @retval         �Ƿ�������ж�������
  * @attention      �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
uint8_t referee_read_data(TimerHandle_t xTimer)
{
    bool_t retval_tf = 0;  //������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
    uint16_t judge_length; //ͳ��һ֡���ݳ���
    uint8_t buff_read[512];

    uint32_t length = fifo_read_buff(pfifo_referee, buff_read, 512);
    if (length)
    {
        Referee.DataLen = length;

		for (uint16_t i = 0; i < length; i++)
		{
			//�ж�֡ͷ�����Ƿ�Ϊ0xA5
			if (buff_read[i] == JUDGE_FRAME_HEADER)
			{
				//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
				memcpy(&FrameHeader, buff_read + i, HeaderLen);
				
				//֡ͷCRC8У��
				if (Verify_CRC8_Check_Sum(buff_read + i, HeaderLen) == 1)
				{
					//ͳ��һ֡���ݳ���,����CR16У��
					judge_length = buff_read[1 + i] + HeaderLen + CmdIdLen + CRC16Len;

					//֡βCRC16У��
					if (Verify_CRC16_Check_Sum(buff_read + i, judge_length) == 1)
					{
						retval_tf = 1; //��У�������˵�����ݿ���

						Referee.Cmd_ID = (buff_read[6 + i] << 8 | buff_read[5 + i]);

						//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
						switch (Referee.Cmd_ID)
						{
	//						case 0x0001: //����״̬����
	//							memcpy(&Referee.GameStatus, (buff_read + DATA), LEN_game_state);
	//							break;

	//						case 0x0002: //�����������
	//							memcpy(&Referee.GmaeResult, (buff_read + DATA), LEN_game_result);
	//							break;

	//						case 0x0003:  //������Ѫ������
	//							memcpy(&Referee.RobotHP, (buff_read + DATA), LEN_game_robot_survivors);
	//							break;

	//						case 0x0004:  //���ڷ���״̬
	//							memcpy(&Referee.DartStatus, (buff_read + DATA), LEN_game_robot_survivors);
	//							break;

	//						case 0x0101:  //�����¼�����
	//							memcpy(&Referee.EventData, (buff_read + DATA), LEN_event_data);
	//							break;

						case 0x0102: //����վ������ʶ
							memcpy(&Referee.SupplyAction, (buff_read + i + DATA), LEN_supply_projectile_action);
							break;

	//						case 0x0104:  //���о�����Ϣ
	//							memcpy(&Referee.RefereeWarning, (buff_read + DATA), LEN_supply_projectile_booking);
	//							break;

	//						case 0x0105:  //���ڷ���ڵ���ʱ
	//							memcpy(&Referee.RemainingTime, (buff_read + DATA), LEN_supply_projectile_booking);
	//							break;

						case 0x0201: //����������״̬
							memcpy(&Referee.RobotStatus, (buff_read + i + DATA), LEN_game_robot_state);
							break;

						case 0x0202: //ʵʱ������������
							memcpy(&Referee.PowerHeat, (buff_read + i + DATA), LEN_power_heat_data);
							break;

						case 0x0203: //������λ��
							memcpy(&Referee.RobotPos, (buff_read + i + DATA), LEN_game_robot_pos);
							break;

	//						case 0x0204:  //����������
	//							memcpy(&Referee.Buff, (buff_read + DATA), LEN_buff_musk);
	//							break;

	//						case 0x0205:  //���л���������״̬
	//							memcpy(&Referee.AerialEnergy, (buff_read + DATA), LEN_aerial_robot_energy);
	//							break;

						case 0x0206: //�˺�״̬
							memcpy(&Referee.RobotHurt, (buff_read + i + DATA), LEN_robot_hurt);

	//							if(RobotHurt.hurt_type == 0)//��װ�װ���������˺�
	//							{
	//								Hurt_Data_Update = 1;	   //װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
	//							}

							break;

						case 0x0207: //ʵʱ�����Ϣ
							memcpy(&Referee.ShootData, (buff_read + i + DATA), LEN_shoot_data);
							break;

						case 0x0208: //�ӵ�ʣ�෢����
							memcpy(&Referee.BulletNum, (buff_read + i + DATA), LEN_bullet_remaining);
							break;

	//						case 0x0209:  //������ RFID ״̬
	//							memcpy(&Referee.RFIDStatus, (buff_read + DATA), LEN_aerial_robot_energy);
	//							break;
						}
						i += FrameHeader.DataLength;
					}
				}

//				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
//				if (*(buff_read + i + sizeof(FrameHeader) + CmdIdLen + FrameHeader.DataLength + CRC16Len) == 0xA5)
//				{
//					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
//					referee_read_data();
//				}
			}			
		}
    }
	else
	{
		//û������
	}
	
	if (pfifo_referee->error)
    {
		// ���մ���
        pfifo_referee->error = 0; 
    }

    return retval_tf; //����������������
}

///********************�������ݸ����жϺ���***************************/

///**
//  * @brief          �����Ƿ����
//  * @param[in]      void
//  * @retval         TRUE����   FALSE������
//  * @attention      �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
//  */
//bool_t JUDGE_sGetDataState(void)
//{
//    return Judge_Data_TF;
//}

///**
//  * @brief          ͳ�Ʒ�����
//  * @param[in]      void
//  * @retval         void
//  * @attention
//  */
//portTickType shoot_time;//������ʱ����
//portTickType shoot_ping;//����������շ����ӳ�
//float Shoot_Speed_Now = 0;
//float Shoot_Speed_Last = 0;
//uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
//void judge_shoot_num_count(void)
//{
//    Shoot_Speed_Now = Referee.ShootData.bullet_speed;

//    if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
//    {
//        ShootNum++;
//        Shoot_Speed_Last = Shoot_Speed_Now;
//    }

////    shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
////    shoot_ping = shoot_time - REVOL_uiGetRevolTime();//�����ӳ�
//}

///**
//  * @brief          ��ȡ������
//  * @param[in]      void
//  * @retval         ������
//  * @attention      ��������˫ǹ��
//  */
//uint16_t judge_usgetshootnum(void)
//{
//    return ShootNum;
//}

///**
//  * @brief          ����������
//  * @param[in]      void
//  * @retval         void
//  * @attention
//  */
//void judge_shootnum_clear(void)
//{
//    ShootNum = 0;
//}

/**
  * @brief          �����������������Ǻ췽
  * @param[in]      void
  * @retval         
  * @attention      1����  0����
  */
uint8_t re_red_or_blue(void)
{
	if (referee_robot_id() >= 1 && referee_robot_id() <= 9)
		return 1; //�Լ��Ǻ췽
	else
		return 0; //�Լ�������
}


/* ID: 0x0102  Byte:  4    ����վ������ʶ */
uint8_t referee_supply_projectile_id(void) //����վ�� ID��
{
    return (Referee.SupplyAction.supply_projectile_id);
}
uint8_t referee_supply_robot_id(void) //���������� ID��
{
    return (Referee.SupplyAction.supply_robot_id);
}
uint8_t referee_supply_projectile_step(void) //�����ڿ���״̬��
{
    return (Referee.SupplyAction.supply_projectile_step);
}
uint8_t referee_supply_projectile_num(void) //����������
{
    return (Referee.SupplyAction.supply_projectile_num);
}

/* ID: 0X0201  Byte: 15    ����������״̬ */
uint8_t referee_robot_id(void) //�������� ID
{
    return (Referee.RobotStatus.robot_id);
}
uint8_t referee_robot_level(void) //�����˵ȼ�
{
    return (Referee.RobotStatus.robot_level);
}
uint16_t referee_remain_HP(void) //������ʣ��Ѫ��
{
    return (Referee.RobotStatus.remain_HP);
}
uint16_t referee_max_HP(void) //����������Ѫ��
{
    return (Referee.RobotStatus.max_HP);
}
uint16_t referee_shooter_id1_17mm_cooling_rate(void) //������ 1 �� 17mm ǹ��ÿ����ȴֵ
{
    return (Referee.RobotStatus.shooter_id1_17mm_cooling_rate);
}
uint16_t referee_shooter_id1_17mm_cooling_limit(void) //������ 1 �� 17mm ǹ����������
{
    return (Referee.RobotStatus.shooter_id1_17mm_cooling_limit);
}
uint16_t referee_shooter_id1_17mm_speed_limit(void) //������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
{
    return (Referee.RobotStatus.shooter_id1_17mm_speed_limit);
}
uint16_t referee_shooter_id2_17mm_cooling_rate(void) //������ 2 �� 17mm ǹ��ÿ����ȴֵ
{
    return (Referee.RobotStatus.shooter_id2_17mm_cooling_rate);
}
uint16_t referee_shooter_id2_17mm_cooling_limit(void) //������ 2 �� 17mm ǹ����������
{
    return (Referee.RobotStatus.shooter_id2_17mm_cooling_limit);
}
uint16_t referee_shooter_id2_17mm_speed_limit(void) //������ 2 �� 17mm ǹ�������ٶ� ��λ m/s
{
    return (Referee.RobotStatus.shooter_id2_17mm_speed_limit);
}
uint16_t referee_shooter_id1_42mm_cooling_rate(void) //������ 42mm ǹ��ÿ����ȴֵ
{
    return (Referee.RobotStatus.shooter_id1_42mm_cooling_rate);
}
uint16_t referee_shooter_id1_42mm_cooling_limit(void) //������ 42mm ǹ����������
{
    return (Referee.RobotStatus.shooter_id1_42mm_cooling_limit);
}
uint16_t referee_shooter_id1_42mm_speed_limit(void) //������ 42mm ǹ�������ٶ� ��λ m/s
{
    return (Referee.RobotStatus.shooter_id1_42mm_speed_limit);
}
uint16_t referee_chassis_power_limit(void) //�����˵��̹�����������
{
    return (Referee.RobotStatus.chassis_power_limit);
}

/* ID: 0X0202  Byte: 14    ʵʱ������������ */
uint16_t referee_chassis_volt(void) //���������ѹ ��λ ����
{
    return (Referee.PowerHeat.chassis_volt);
}
uint16_t referee_chassis_current(void) //����������� ��λ ����
{
    return (Referee.PowerHeat.chassis_current);
}
float referee_chassis_power(void) //����������� ��λ W ��
{
    return (Referee.PowerHeat.chassis_power);
}
uint16_t referee_chassis_power_buffer(void) //���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
{
    return (Referee.PowerHeat.chassis_power_buffer);
}
uint16_t referee_shooter_id1_17mm_cooling_heat(void) //1 �� 17mm ǹ������
{
    return (Referee.PowerHeat.shooter_id1_17mm_cooling_heat);
}
uint16_t referee_shooter_id2_17mm_cooling_heat(void) //2 �� 17mm ǹ������
{
    return (Referee.PowerHeat.shooter_id2_17mm_cooling_heat);
}
uint16_t referee_shooter_id1_42mm_cooling_heat(void) //42mm ǹ������
{
    return (Referee.PowerHeat.shooter_id1_42mm_cooling_heat);
}

/* ID: 0x0203  Byte: 16    ������λ������ */
float referee_yaw(void) //λ��ǹ�ڣ���λ��
{
    return (Referee.RobotPos.yaw);
}

/* ID: 0x0206  Byte:  1    �˺�״̬���� */
uint8_t referee_armor_id(void) //bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0
{
    return (Referee.RobotHurt.armor_id);
}
uint8_t referee_hurt_type(void) //bit 4-7��Ѫ���仯����
{
    /*
	0x0 װ���˺���Ѫ��
	0x1 ģ����߿�Ѫ��
	0x2 �����ٿ�Ѫ��
	0x3 ��ǹ��������Ѫ��
	0x4 �����̹��ʿ�Ѫ��
	0x5 װ��ײ����Ѫ��
	*/
    return (Referee.RobotHurt.hurt_type);
}

/* ID: 0x0207  Byte:  6    ʵʱ������� */
uint8_t bullet_type(void) //�ӵ�����: 1��17mm ���� 2��42mm ����
{
    return (Referee.ShootData.bullet_type);
}
uint8_t shooter_id(void) //������� ID��
{
    return (Referee.ShootData.shooter_id);
}
uint8_t bullet_freq(void) //�ӵ���Ƶ ��λ Hz
{
    return (Referee.ShootData.bullet_freq);
}
float bullet_speed(void) //�ӵ����� ��λ m/s
{
    return (Referee.ShootData.bullet_speed);
}

/* ID: 0x0208  Byte:  2    ʵʱ������� */
uint16_t bullet_remaining_num_17mm(void) //17mm �ӵ�ʣ�෢����Ŀ
{
    return (Referee.BulletNum.bullet_remaining_num_17mm);
}
uint16_t bullet_remaining_num_42mm(void) //42mm �ӵ�ʣ�෢����Ŀ
{
    return (Referee.BulletNum.bullet_remaining_num_42mm);
}
uint16_t coin_remaining_num(void) //ʣ��������
{
    return (Referee.BulletNum.coin_remaining_num);
}
