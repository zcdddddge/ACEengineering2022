#ifndef __REFEREEDEAL_H_
#define __REFEREEDEAL_H_
#include "struct_typedef.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "chassis_task.h"


/***************命令码ID********************/

/*

	ID: 0x0001  Byte:  3    比赛状态数据       			发送频率 1Hz
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送
	ID: 0x0003  Byte: 32    比赛机器人存活数据   		1Hz发送
	ID: 0x0004  Byte:  3    飞镖发射状态                飞镖发射后发送，发送范围：所有机器人
	ID: 0x0101  Byte:  4    场地事件数据   				1Hz 周期发送
	ID: 0x0102  Byte:  4    场地补给站动作标识数据    	动作触发后发送
	ID: 0x0104  Byte:  2    裁判警告信息                警告发生后发送
	ID: 0X0105  Byte:  1    飞镖发射口倒计时            1Hz 周期发送
	ID: 0X0201  Byte: 18    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 16    实时功率热量数据   			50Hz
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	1Hz 周期发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		射击后发送
	ID: 0x0208  Byte:  2    子弹剩余发射数              1Hz 周期发送
	ID: 0x0208  Byte:  4    机器人 RFID 状态            1Hz
	ID: 0x020A  Byte: 12    飞镖机器人客户端指令数据    10Hz
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz

*/
//命令码ID,用来判断接收的是什么数据
typedef enum
{
    ID_game_state       			= 0x0001,//比赛状态数据
    ID_game_result 	   				= 0x0002,//比赛结果数据
    ID_game_robot_survivors       	= 0x0003,//比赛机器人存活数据
    ID_event_data  					= 0x0101,//场地事件数据
    ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
    ID_supply_projectile_booking 	= 0x0103,//场地补给站预约子弹数据
    ID_game_robot_state    			= 0x0201,//机器人状态数据
    ID_power_heat_data    			= 0x0202,//实时功率热量数据
    ID_game_robot_pos        		= 0x0203,//机器人位置数据
    ID_buff_musk					= 0x0204,//机器人增益数据
    ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
    ID_robot_hurt					= 0x0206,//伤害状态数据
    ID_shoot_data					= 0x0207,//实时射击数据

} CmdID;

//命令码数据段长,根据官方协议来定义长度
typedef enum
{
    LEN_game_state       				=  3,	//0x0001
    LEN_game_result       				=  1,	//0x0002
    LEN_game_robot_survivors       		= 32,	//0x0003
    LEN_event_data  					=  4,	//0x0101
    LEN_supply_projectile_action        =  4,	//0x0102  - 2021.4.30更新
    LEN_supply_projectile_booking		=  2,	//0x0103
    LEN_game_robot_state    			= 27,	//0x0201  - 2021.4.30更新
    LEN_power_heat_data   				= 16,	//0x0202  - 2021.4.30更新
    LEN_game_robot_pos        			= 16,	//0x0203  - 2021.4.30更新
    LEN_buff_musk        				=  1,	//0x0204
    LEN_aerial_robot_energy        		=  3,	//0x0205
    LEN_robot_hurt        				=  1,	//0x0206  - 2021.4.30更新
    LEN_shoot_data       				=  7,	//0x0207  - 2021.4.30更新
	LEN_bullet_remaining                =  6,   //0x0208  - 2021.4.30更新
	LEN_rfid_status                     =  4,   //0x0209
	LEN_dart_client_cmd                 = 12,   //0x020A
} JudgeDataLength;

/* 自定义帧头 */
typedef __packed struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;
} xFrameHeader_t;

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)
typedef enum
{
    FRAME_HEADER = 0,
    CMD_ID = 5,
    DATA = 7,

} JudgeFrameOffset;


typedef enum
{
    RED_HERO = 1,        //英雄
    RED_ENGINEER = 2,    //工程
    RED_STANDARD_1 = 3,  //步兵
    RED_STANDARD_2 = 4,  //步兵
    RED_STANDARD_3 = 5,  //步兵
    RED_AERIAL = 6,      //空中
    RED_SENTRY = 7,      //哨兵
    RED_DARTS = 8,       //飞镖
    RED_RADAR = 9,       //雷达

    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
    BLUE_DARTS = 108,
    BLUE_RADAR = 109,
} robot_id_e;



/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint8_t error;
} ext_game_status_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    机器人血量数据 */
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
    uint8_t error;
} ext_game_robot_HP_t;

/* ID: 0x0004  Byte:  3    飞镖发射状态 */
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
    uint8_t error;
} ext_dart_status_t;


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct
{
    uint32_t Parking : 2;
    uint32_t EnergyAgency : 2;
    uint32_t Shield : 1;
    uint32_t other : 27;
    uint8_t error;
} ext_event_data_t;

/* ID: 0x0102  Byte:  4    补给站动作标识 */
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
	
    uint8_t error;
} ext_supply_projectile_action_t;

/* ID: 0x0104  Byte:  2    裁判警告信息 */
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

/* ID: 0x0105  Byte:  1    飞镖发射口倒计时 */
typedef __packed struct
{
    uint8_t dart_remaining_time;
    uint8_t error;
} ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 15    比赛机器人状态 */
typedef __packed struct
{
    uint8_t robot_id;                        //本机器人 ID
    uint8_t robot_level;                     //机器人等级
    uint16_t remain_HP;                      //机器人剩余血量
    uint16_t max_HP;                         //机器人上限血量
    uint16_t shooter_id1_17mm_cooling_rate;  //机器人 1 号 17mm 枪口每秒冷却值
    uint16_t shooter_id1_17mm_cooling_limit; //机器人 1 号 17mm 枪口热量上限
    uint16_t shooter_id1_17mm_speed_limit;   //机器人 1 号 17mm 枪口上限速度 单位 m/s
    uint16_t shooter_id2_17mm_cooling_rate;  //机器人 2 号 17mm 枪口每秒冷却值
    uint16_t shooter_id2_17mm_cooling_limit; //机器人 2 号 17mm 枪口热量上限
    uint16_t shooter_id2_17mm_speed_limit;   //机器人 2 号 17mm 枪口上限速度 单位 m/s
    uint16_t shooter_id1_42mm_cooling_rate;  //机器人 42mm 枪口每秒冷却值
    uint16_t shooter_id1_42mm_cooling_limit; //机器人 42mm 枪口热量上限
    uint16_t shooter_id1_42mm_speed_limit;   //机器人 42mm 枪口上限速度 单位 m/s
    uint16_t chassis_power_limit;            //机器人底盘功率限制上限
    uint8_t mains_power_gimbal_output : 1;   //0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
    uint8_t mains_power_chassis_output : 1;  //1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
    uint8_t mains_power_shooter_output : 1;  //2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
    //uint8_t error;
} ext_game_robot_status_t;

/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct
{
    uint16_t chassis_volt;                  //底盘输出电压 单位 毫伏
    uint16_t chassis_current;               //底盘输出电流 单位 毫安
    float chassis_power;                    //底盘输出功率 单位 W 瓦
    uint16_t chassis_power_buffer;          //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
    uint16_t shooter_id1_17mm_cooling_heat; //1 号 17mm 枪口热量
    uint16_t shooter_id2_17mm_cooling_heat; //2 号 17mm 枪口热量
    uint16_t shooter_id1_42mm_cooling_heat; //42mm 枪口热量
    //uint8_t error;
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct
{
    float x;   //位置 x 坐标，单位 m
    float y;   //位置 y 坐标，单位 m
    float z;   //位置 z 坐标，单位 m
    float yaw; //位置枪口，单位度
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct
{
    uint16_t energy_point;
    uint8_t attack_time;
    uint8_t error;
} ext_aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct
{
    uint8_t armor_id : 4;  //bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0
    uint8_t hurt_type : 4; //bit 4-7：血量变化类型
	/*
	0x0 装甲伤害扣血；
	0x1 模块掉线扣血；
	0x2 超射速扣血；
	0x3 超枪口热量扣血；
	0x4 超底盘功率扣血；
	0x5 装甲撞击扣血
	*/
    uint8_t error;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct
{
	uint8_t bullet_type; //子弹类型: 1：17mm 弹丸 2：42mm 弹丸
	uint8_t shooter_id;  //发射机构 ID：
	/*
	1：1 号 17mm 发射机构
	2：2 号 17mm 发射机构
	3：42mm 发射机构
	*/
	uint8_t bullet_freq; //子弹射频 单位 Hz
	float bullet_speed;  //子弹射速 单位 m/s
    uint8_t error;
} ext_shoot_data_t;

/* ID: 0x0208  Byte:  2    实时射击数据 */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm; //17mm 子弹剩余发射数目
	uint16_t bullet_remaining_num_42mm; //42mm 子弹剩余发射数目
	uint16_t coin_remaining_num;        //剩余金币数量
    uint8_t error;
} ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  4    机器人 RFID 状态 */
typedef __packed struct
{
    uint32_t BaseLand : 1;
    uint32_t HighLand : 1;
    uint32_t Energy : 1;
    uint32_t OverSlope : 1;
    uint32_t Sentry : 1;
    uint32_t Resources : 1;
    uint32_t BloodPoint : 1;
    uint32_t BloodCard : 1;
    uint32_t other : 24;
    uint8_t error;
} ext_rfid_status_t;

typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 裁判系统数据，总结构体 */
typedef __packed struct
{
    //uint8_t RefereeData[256];
    //uint8_t RealData[45];

    int16_t DataLen;
    int16_t RealLen;
    int Cmd_ID;
    uint8_t RECEIVE_FLAG;

    ext_game_status_t GameStatus; //0x0001
    ext_game_result_t GmaeResult; //0x0002
    ext_game_robot_HP_t RobotHP;  //0x0003
    ext_dart_status_t DartStatus; //0x0004
    ext_event_data_t EventData;   //0x0101
    ext_supply_projectile_action_t SupplyAction;  //0x0102
    ext_referee_warning_t RefereeWarning;  //0x0104
    ext_dart_remaining_time_t RemainingTime;  //0x0105
    ext_game_robot_status_t RobotStatus;  //0x0201
    ext_power_heat_data_t PowerHeat;  //0x0202
    ext_game_robot_pos_t RobotPos;  //0x0203
    ext_buff_musk_t Buff;  //0x0204
    ext_aerial_robot_energy_t AerialEnergy;  //0x0205
    ext_robot_hurt_t RobotHurt;  //0x0206
    ext_shoot_data_t ShootData;  //0x0207
    ext_bullet_remaining_t BulletNum;   //0x0208
    ext_rfid_status_t RFIDStatus;  //0x0209
} REFEREE_t;


/*float数组重组*/
typedef union
{
    struct
    {
        uint8_t LB;
        uint8_t MLB;
        uint8_t MHB;
        uint8_t HB;
    } float_byte;
    float value;
} Float_t;




extern void referee_system_init(void); //串口六初始化
REFEREE_t *return_referee_point(void); //返回函数指针
extern uint8_t referee_read_data(TimerHandle_t xTimer);
uint8_t re_red_or_blue(void);

//返回裁判系统数据
/* ID: 0x0102  Byte:  4    补给站动作标识 */
uint8_t referee_supply_projectile_id(void); //补给站口 ID：
uint8_t referee_supply_robot_id(void); //补弹机器人 ID：
uint8_t referee_supply_projectile_step(void); //出弹口开闭状态：
uint8_t referee_supply_projectile_num(void); //补弹数量：
/* ID: 0X0201  Byte: 15    比赛机器人状态 */
uint8_t referee_robot_id(void); //本机器人 ID
uint8_t referee_robot_level(void); //机器人等级
uint16_t referee_remain_HP(void); //机器人剩余血量
uint16_t referee_max_HP(void); //机器人上限血量
uint16_t referee_shooter_id1_17mm_cooling_rate(void); //机器人 1 号 17mm 枪口每秒冷却值
uint16_t referee_shooter_id1_17mm_cooling_limit(void); //机器人 1 号 17mm 枪口热量上限
uint16_t referee_shooter_id1_17mm_speed_limit(void); //机器人 1 号 17mm 枪口上限速度 单位 m/s
uint16_t referee_shooter_id2_17mm_cooling_rate(void); //机器人 2 号 17mm 枪口每秒冷却值
uint16_t referee_shooter_id2_17mm_cooling_limit(void); //机器人 2 号 17mm 枪口热量上限
uint16_t referee_shooter_id2_17mm_speed_limit(void); //机器人 2 号 17mm 枪口上限速度 单位 m/s
uint16_t referee_shooter_id1_42mm_cooling_rate(void); //机器人 42mm 枪口每秒冷却值
uint16_t referee_shooter_id1_42mm_cooling_limit(void); //机器人 42mm 枪口热量上限
uint16_t referee_shooter_id1_42mm_speed_limit(void); //机器人 42mm 枪口上限速度 单位 m/s
uint16_t referee_chassis_power_limit(void); //机器人底盘功率限制上限
/* ID: 0X0202  Byte: 14    实时功率热量数据 */
uint16_t referee_chassis_volt(void); //底盘输出电压 单位 毫伏
uint16_t referee_chassis_current(void); //底盘输出电流 单位 毫安
float referee_chassis_power(void); //底盘输出功率 单位 W 瓦
uint16_t referee_chassis_power_buffer(void); //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
uint16_t referee_shooter_id1_17mm_cooling_heat(void); //1 号 17mm 枪口热量
uint16_t referee_shooter_id2_17mm_cooling_heat(void); //2 号 17mm 枪口热量
uint16_t referee_shooter_id1_42mm_cooling_heat(void); //42mm 枪口热量
/* ID: 0x0203  Byte: 16    机器人位置数据 */
float referee_yaw(void); //位置枪口，单位度
/* ID: 0x0206  Byte:  1    伤害状态数据 */
uint8_t referee_armor_id(void); //bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0
uint8_t referee_hurt_type(void); //bit 4-7：血量变化类型
/* ID: 0x0207  Byte:  6    实时射击数据 */
uint8_t bullet_type(void); //子弹类型: 1：17mm 弹丸 2：42mm 弹丸
uint8_t shooter_id(void); //发射机构 ID：
uint8_t bullet_freq(void); //子弹射频 单位 Hz
float bullet_speed(void); //子弹射速 单位 m/s
/* ID: 0x0208  Byte:  2    实时射击数据 */
uint16_t bullet_remaining_num_17mm(void); //17mm 子弹剩余发射数目
uint16_t bullet_remaining_num_42mm(void); //42mm 子弹剩余发射数目
uint16_t coin_remaining_num(void); //剩余金币数量
	
#endif
