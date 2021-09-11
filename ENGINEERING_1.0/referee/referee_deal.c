#include "referee_deal.h"
#include "usart6.h"
#include "uart4.h"
#include "crc.h"
#include "parameter.h"
#include "fifo_buff.h"

//static
REFEREE_t Referee;          //裁判系统数据结构体
xFrameHeader_t FrameHeader; //发送帧头信息

extern fifo_rx_def fifo_uart_rx_4;
fifo_rx_def *pfifo_referee = &fifo_uart_rx_4;

/**
  * @brief          串口六初始化
  * @param[in]      none
  * @retval         none
  * @attention
  */
void referee_system_init(void)
{
    uart4_init(5); //裁判系统
}

/**
  * @brief          获得裁判系统数据控制量
  * @param[in]      none
  * @retval         裁判系统数据控制指针 &Referee
  * @attention
  */
REFEREE_t *return_referee_point(void)
{
    return &Referee;
}

/*数据段长度*/
static const uint8_t HeaderLen = 5;
static const uint8_t CmdIdLen = 2;
static const uint8_t CRC16Len = 2;

bool_t Judge_Data_TF = 0; //裁判数据是否可用,辅助函数调用

/**
  * @brief          读取裁判数据,中断中读取保证速度
  * @param[in]      接收裁判系统数据结构体,缓存数据
  * @retval         是否对正误判断做处理
  * @attention      在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
uint8_t referee_read_data(TimerHandle_t xTimer)
{
    bool_t retval_tf = 0;  //数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
    uint16_t judge_length; //统计一帧数据长度
    uint8_t buff_read[512];

    uint32_t length = fifo_read_buff(pfifo_referee, buff_read, 512);
    if (length)
    {
        Referee.DataLen = length;

		for (uint16_t i = 0; i < length; i++)
		{
			//判断帧头数据是否为0xA5
			if (buff_read[i] == JUDGE_FRAME_HEADER)
			{
				//写入帧头数据,用于判断是否开始存储裁判数据
				memcpy(&FrameHeader, buff_read + i, HeaderLen);
				
				//帧头CRC8校验
				if (Verify_CRC8_Check_Sum(buff_read + i, HeaderLen) == 1)
				{
					//统计一帧数据长度,用于CR16校验
					judge_length = buff_read[1 + i] + HeaderLen + CmdIdLen + CRC16Len;

					//帧尾CRC16校验
					if (Verify_CRC16_Check_Sum(buff_read + i, judge_length) == 1)
					{
						retval_tf = 1; //都校验过了则说明数据可用

						Referee.Cmd_ID = (buff_read[6 + i] << 8 | buff_read[5 + i]);

						//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
						switch (Referee.Cmd_ID)
						{
	//						case 0x0001: //比赛状态数据
	//							memcpy(&Referee.GameStatus, (buff_read + DATA), LEN_game_state);
	//							break;

	//						case 0x0002: //比赛结果数据
	//							memcpy(&Referee.GmaeResult, (buff_read + DATA), LEN_game_result);
	//							break;

	//						case 0x0003:  //机器人血量数据
	//							memcpy(&Referee.RobotHP, (buff_read + DATA), LEN_game_robot_survivors);
	//							break;

	//						case 0x0004:  //飞镖发射状态
	//							memcpy(&Referee.DartStatus, (buff_read + DATA), LEN_game_robot_survivors);
	//							break;

	//						case 0x0101:  //场地事件数据
	//							memcpy(&Referee.EventData, (buff_read + DATA), LEN_event_data);
	//							break;

						case 0x0102: //补给站动作标识
							memcpy(&Referee.SupplyAction, (buff_read + i + DATA), LEN_supply_projectile_action);
							break;

	//						case 0x0104:  //裁判警告信息
	//							memcpy(&Referee.RefereeWarning, (buff_read + DATA), LEN_supply_projectile_booking);
	//							break;

	//						case 0x0105:  //飞镖发射口倒计时
	//							memcpy(&Referee.RemainingTime, (buff_read + DATA), LEN_supply_projectile_booking);
	//							break;

						case 0x0201: //比赛机器人状态
							memcpy(&Referee.RobotStatus, (buff_read + i + DATA), LEN_game_robot_state);
							break;

						case 0x0202: //实时功率热量数据
							memcpy(&Referee.PowerHeat, (buff_read + i + DATA), LEN_power_heat_data);
							break;

						case 0x0203: //机器人位置
							memcpy(&Referee.RobotPos, (buff_read + i + DATA), LEN_game_robot_pos);
							break;

	//						case 0x0204:  //机器人增益
	//							memcpy(&Referee.Buff, (buff_read + DATA), LEN_buff_musk);
	//							break;

	//						case 0x0205:  //空中机器人能量状态
	//							memcpy(&Referee.AerialEnergy, (buff_read + DATA), LEN_aerial_robot_energy);
	//							break;

						case 0x0206: //伤害状态
							memcpy(&Referee.RobotHurt, (buff_read + i + DATA), LEN_robot_hurt);

	//							if(RobotHurt.hurt_type == 0)//非装甲板离线造成伤害
	//							{
	//								Hurt_Data_Update = 1;	   //装甲数据每更新一次则判定为受到一次伤害
	//							}

							break;

						case 0x0207: //实时射击信息
							memcpy(&Referee.ShootData, (buff_read + i + DATA), LEN_shoot_data);
							break;

						case 0x0208: //子弹剩余发射数
							memcpy(&Referee.BulletNum, (buff_read + i + DATA), LEN_bullet_remaining);
							break;

	//						case 0x0209:  //机器人 RFID 状态
	//							memcpy(&Referee.RFIDStatus, (buff_read + DATA), LEN_aerial_robot_energy);
	//							break;
						}
						i += FrameHeader.DataLength;
					}
				}

//				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
//				if (*(buff_read + i + sizeof(FrameHeader) + CmdIdLen + FrameHeader.DataLength + CRC16Len) == 0xA5)
//				{
//					//如果一个数据包出现了多帧数据,则再次读取
//					referee_read_data();
//				}
			}			
		}
    }
	else
	{
		//没有数据
	}
	
	if (pfifo_referee->error)
    {
		// 接收错误
        pfifo_referee->error = 0; 
    }

    return retval_tf; //对数据正误做处理
}

///********************裁判数据辅助判断函数***************************/

///**
//  * @brief          数据是否可用
//  * @param[in]      void
//  * @retval         TRUE可用   FALSE不可用
//  * @attention      在裁判读取函数中实时改变返回值
//  */
//bool_t JUDGE_sGetDataState(void)
//{
//    return Judge_Data_TF;
//}

///**
//  * @brief          统计发弹量
//  * @param[in]      void
//  * @retval         void
//  * @attention
//  */
//portTickType shoot_time;//发射延时测试
//portTickType shoot_ping;//计算出的最终发弹延迟
//float Shoot_Speed_Now = 0;
//float Shoot_Speed_Last = 0;
//uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
//void judge_shoot_num_count(void)
//{
//    Shoot_Speed_Now = Referee.ShootData.bullet_speed;

//    if(Shoot_Speed_Last != Shoot_Speed_Now)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
//    {
//        ShootNum++;
//        Shoot_Speed_Last = Shoot_Speed_Now;
//    }

////    shoot_time = xTaskGetTickCount();//获取弹丸发射时的系统时间
////    shoot_ping = shoot_time - REVOL_uiGetRevolTime();//计算延迟
//}

///**
//  * @brief          读取发弹量
//  * @param[in]      void
//  * @retval         发弹量
//  * @attention      不适用于双枪管
//  */
//uint16_t judge_usgetshootnum(void)
//{
//    return ShootNum;
//}

///**
//  * @brief          发弹量清零
//  * @param[in]      void
//  * @retval         void
//  * @attention
//  */
//void judge_shootnum_clear(void)
//{
//    ShootNum = 0;
//}

/**
  * @brief          返回自身是蓝方还是红方
  * @param[in]      void
  * @retval         
  * @attention      1：红  0：蓝
  */
uint8_t re_red_or_blue(void)
{
	if (referee_robot_id() >= 1 && referee_robot_id() <= 9)
		return 1; //自己是红方
	else
		return 0; //自己是蓝方
}


/* ID: 0x0102  Byte:  4    补给站动作标识 */
uint8_t referee_supply_projectile_id(void) //补给站口 ID：
{
    return (Referee.SupplyAction.supply_projectile_id);
}
uint8_t referee_supply_robot_id(void) //补弹机器人 ID：
{
    return (Referee.SupplyAction.supply_robot_id);
}
uint8_t referee_supply_projectile_step(void) //出弹口开闭状态：
{
    return (Referee.SupplyAction.supply_projectile_step);
}
uint8_t referee_supply_projectile_num(void) //补弹数量：
{
    return (Referee.SupplyAction.supply_projectile_num);
}

/* ID: 0X0201  Byte: 15    比赛机器人状态 */
uint8_t referee_robot_id(void) //本机器人 ID
{
    return (Referee.RobotStatus.robot_id);
}
uint8_t referee_robot_level(void) //机器人等级
{
    return (Referee.RobotStatus.robot_level);
}
uint16_t referee_remain_HP(void) //机器人剩余血量
{
    return (Referee.RobotStatus.remain_HP);
}
uint16_t referee_max_HP(void) //机器人上限血量
{
    return (Referee.RobotStatus.max_HP);
}
uint16_t referee_shooter_id1_17mm_cooling_rate(void) //机器人 1 号 17mm 枪口每秒冷却值
{
    return (Referee.RobotStatus.shooter_id1_17mm_cooling_rate);
}
uint16_t referee_shooter_id1_17mm_cooling_limit(void) //机器人 1 号 17mm 枪口热量上限
{
    return (Referee.RobotStatus.shooter_id1_17mm_cooling_limit);
}
uint16_t referee_shooter_id1_17mm_speed_limit(void) //机器人 1 号 17mm 枪口上限速度 单位 m/s
{
    return (Referee.RobotStatus.shooter_id1_17mm_speed_limit);
}
uint16_t referee_shooter_id2_17mm_cooling_rate(void) //机器人 2 号 17mm 枪口每秒冷却值
{
    return (Referee.RobotStatus.shooter_id2_17mm_cooling_rate);
}
uint16_t referee_shooter_id2_17mm_cooling_limit(void) //机器人 2 号 17mm 枪口热量上限
{
    return (Referee.RobotStatus.shooter_id2_17mm_cooling_limit);
}
uint16_t referee_shooter_id2_17mm_speed_limit(void) //机器人 2 号 17mm 枪口上限速度 单位 m/s
{
    return (Referee.RobotStatus.shooter_id2_17mm_speed_limit);
}
uint16_t referee_shooter_id1_42mm_cooling_rate(void) //机器人 42mm 枪口每秒冷却值
{
    return (Referee.RobotStatus.shooter_id1_42mm_cooling_rate);
}
uint16_t referee_shooter_id1_42mm_cooling_limit(void) //机器人 42mm 枪口热量上限
{
    return (Referee.RobotStatus.shooter_id1_42mm_cooling_limit);
}
uint16_t referee_shooter_id1_42mm_speed_limit(void) //机器人 42mm 枪口上限速度 单位 m/s
{
    return (Referee.RobotStatus.shooter_id1_42mm_speed_limit);
}
uint16_t referee_chassis_power_limit(void) //机器人底盘功率限制上限
{
    return (Referee.RobotStatus.chassis_power_limit);
}

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
uint16_t referee_chassis_volt(void) //底盘输出电压 单位 毫伏
{
    return (Referee.PowerHeat.chassis_volt);
}
uint16_t referee_chassis_current(void) //底盘输出电流 单位 毫安
{
    return (Referee.PowerHeat.chassis_current);
}
float referee_chassis_power(void) //底盘输出功率 单位 W 瓦
{
    return (Referee.PowerHeat.chassis_power);
}
uint16_t referee_chassis_power_buffer(void) //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
{
    return (Referee.PowerHeat.chassis_power_buffer);
}
uint16_t referee_shooter_id1_17mm_cooling_heat(void) //1 号 17mm 枪口热量
{
    return (Referee.PowerHeat.shooter_id1_17mm_cooling_heat);
}
uint16_t referee_shooter_id2_17mm_cooling_heat(void) //2 号 17mm 枪口热量
{
    return (Referee.PowerHeat.shooter_id2_17mm_cooling_heat);
}
uint16_t referee_shooter_id1_42mm_cooling_heat(void) //42mm 枪口热量
{
    return (Referee.PowerHeat.shooter_id1_42mm_cooling_heat);
}

/* ID: 0x0203  Byte: 16    机器人位置数据 */
float referee_yaw(void) //位置枪口，单位度
{
    return (Referee.RobotPos.yaw);
}

/* ID: 0x0206  Byte:  1    伤害状态数据 */
uint8_t referee_armor_id(void) //bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0
{
    return (Referee.RobotHurt.armor_id);
}
uint8_t referee_hurt_type(void) //bit 4-7：血量变化类型
{
    /*
	0x0 装甲伤害扣血；
	0x1 模块掉线扣血；
	0x2 超射速扣血；
	0x3 超枪口热量扣血；
	0x4 超底盘功率扣血；
	0x5 装甲撞击扣血：
	*/
    return (Referee.RobotHurt.hurt_type);
}

/* ID: 0x0207  Byte:  6    实时射击数据 */
uint8_t bullet_type(void) //子弹类型: 1：17mm 弹丸 2：42mm 弹丸
{
    return (Referee.ShootData.bullet_type);
}
uint8_t shooter_id(void) //发射机构 ID：
{
    return (Referee.ShootData.shooter_id);
}
uint8_t bullet_freq(void) //子弹射频 单位 Hz
{
    return (Referee.ShootData.bullet_freq);
}
float bullet_speed(void) //子弹射速 单位 m/s
{
    return (Referee.ShootData.bullet_speed);
}

/* ID: 0x0208  Byte:  2    实时射击数据 */
uint16_t bullet_remaining_num_17mm(void) //17mm 子弹剩余发射数目
{
    return (Referee.BulletNum.bullet_remaining_num_17mm);
}
uint16_t bullet_remaining_num_42mm(void) //42mm 子弹剩余发射数目
{
    return (Referee.BulletNum.bullet_remaining_num_42mm);
}
uint16_t coin_remaining_num(void) //剩余金币数量
{
    return (Referee.BulletNum.coin_remaining_num);
}
