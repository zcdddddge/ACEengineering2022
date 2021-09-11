#ifndef __REFEREEDEAL_H_
#define __REFEREEDEAL_H_
#include "struct_typedef.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "chassis_task.h"


/***************������ID********************/

/*

	ID: 0x0001  Byte:  3    ����״̬����       			����Ƶ�� 1Hz
	ID: 0x0002  Byte:  1    �����������         		������������
	ID: 0x0003  Byte: 32    ���������˴������   		1Hz����
	ID: 0x0004  Byte:  3    ���ڷ���״̬                ���ڷ�����ͣ����ͷ�Χ�����л�����
	ID: 0x0101  Byte:  4    �����¼�����   				1Hz ���ڷ���
	ID: 0x0102  Byte:  4    ���ز���վ������ʶ����    	������������
	ID: 0x0104  Byte:  2    ���о�����Ϣ                ���淢������
	ID: 0X0105  Byte:  1    ���ڷ���ڵ���ʱ            1Hz ���ڷ���
	ID: 0X0201  Byte: 18    ������״̬����        		10Hz
	ID: 0X0202  Byte: 16    ʵʱ������������   			50Hz
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	1Hz ���ڷ���
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�������
	ID: 0x0208  Byte:  2    �ӵ�ʣ�෢����              1Hz ���ڷ���
	ID: 0x0208  Byte:  4    ������ RFID ״̬            1Hz
	ID: 0x020A  Byte: 12    ���ڻ����˿ͻ���ָ������    10Hz
	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz

*/
//������ID,�����жϽ��յ���ʲô����
typedef enum
{
    ID_game_state       			= 0x0001,//����״̬����
    ID_game_result 	   				= 0x0002,//�����������
    ID_game_robot_survivors       	= 0x0003,//���������˴������
    ID_event_data  					= 0x0101,//�����¼�����
    ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
    ID_supply_projectile_booking 	= 0x0103,//���ز���վԤԼ�ӵ�����
    ID_game_robot_state    			= 0x0201,//������״̬����
    ID_power_heat_data    			= 0x0202,//ʵʱ������������
    ID_game_robot_pos        		= 0x0203,//������λ������
    ID_buff_musk					= 0x0204,//��������������
    ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
    ID_robot_hurt					= 0x0206,//�˺�״̬����
    ID_shoot_data					= 0x0207,//ʵʱ�������

} CmdID;

//���������ݶγ�,���ݹٷ�Э�������峤��
typedef enum
{
    LEN_game_state       				=  3,	//0x0001
    LEN_game_result       				=  1,	//0x0002
    LEN_game_robot_survivors       		= 32,	//0x0003
    LEN_event_data  					=  4,	//0x0101
    LEN_supply_projectile_action        =  4,	//0x0102  - 2021.4.30����
    LEN_supply_projectile_booking		=  2,	//0x0103
    LEN_game_robot_state    			= 27,	//0x0201  - 2021.4.30����
    LEN_power_heat_data   				= 16,	//0x0202  - 2021.4.30����
    LEN_game_robot_pos        			= 16,	//0x0203  - 2021.4.30����
    LEN_buff_musk        				=  1,	//0x0204
    LEN_aerial_robot_energy        		=  3,	//0x0205
    LEN_robot_hurt        				=  1,	//0x0206  - 2021.4.30����
    LEN_shoot_data       				=  7,	//0x0207  - 2021.4.30����
	LEN_bullet_remaining                =  6,   //0x0208  - 2021.4.30����
	LEN_rfid_status                     =  4,   //0x0209
	LEN_dart_client_cmd                 = 12,   //0x020A
} JudgeDataLength;

/* �Զ���֡ͷ */
typedef __packed struct
{
    uint8_t  SOF;
    uint16_t DataLength;
    uint8_t  Seq;
    uint8_t  CRC8;
} xFrameHeader_t;

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)
typedef enum
{
    FRAME_HEADER = 0,
    CMD_ID = 5,
    DATA = 7,

} JudgeFrameOffset;


typedef enum
{
    RED_HERO = 1,        //Ӣ��
    RED_ENGINEER = 2,    //����
    RED_STANDARD_1 = 3,  //����
    RED_STANDARD_2 = 4,  //����
    RED_STANDARD_3 = 5,  //����
    RED_AERIAL = 6,      //����
    RED_SENTRY = 7,      //�ڱ�
    RED_DARTS = 8,       //����
    RED_RADAR = 9,       //�״�

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



/* ID: 0x0001  Byte:  3    ����״̬���� */
typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint8_t error;
} ext_game_status_t;

/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    ������Ѫ������ */
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

/* ID: 0x0004  Byte:  3    ���ڷ���״̬ */
typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
    uint8_t error;
} ext_dart_status_t;


/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct
{
    uint32_t Parking : 2;
    uint32_t EnergyAgency : 2;
    uint32_t Shield : 1;
    uint32_t other : 27;
    uint8_t error;
} ext_event_data_t;

/* ID: 0x0102  Byte:  4    ����վ������ʶ */
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
	
    uint8_t error;
} ext_supply_projectile_action_t;

/* ID: 0x0104  Byte:  2    ���о�����Ϣ */
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

/* ID: 0x0105  Byte:  1    ���ڷ���ڵ���ʱ */
typedef __packed struct
{
    uint8_t dart_remaining_time;
    uint8_t error;
} ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 15    ����������״̬ */
typedef __packed struct
{
    uint8_t robot_id;                        //�������� ID
    uint8_t robot_level;                     //�����˵ȼ�
    uint16_t remain_HP;                      //������ʣ��Ѫ��
    uint16_t max_HP;                         //����������Ѫ��
    uint16_t shooter_id1_17mm_cooling_rate;  //������ 1 �� 17mm ǹ��ÿ����ȴֵ
    uint16_t shooter_id1_17mm_cooling_limit; //������ 1 �� 17mm ǹ����������
    uint16_t shooter_id1_17mm_speed_limit;   //������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
    uint16_t shooter_id2_17mm_cooling_rate;  //������ 2 �� 17mm ǹ��ÿ����ȴֵ
    uint16_t shooter_id2_17mm_cooling_limit; //������ 2 �� 17mm ǹ����������
    uint16_t shooter_id2_17mm_speed_limit;   //������ 2 �� 17mm ǹ�������ٶ� ��λ m/s
    uint16_t shooter_id1_42mm_cooling_rate;  //������ 42mm ǹ��ÿ����ȴֵ
    uint16_t shooter_id1_42mm_cooling_limit; //������ 42mm ǹ����������
    uint16_t shooter_id1_42mm_speed_limit;   //������ 42mm ǹ�������ٶ� ��λ m/s
    uint16_t chassis_power_limit;            //�����˵��̹�����������
    uint8_t mains_power_gimbal_output : 1;   //0 bit��gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v �����
    uint8_t mains_power_chassis_output : 1;  //1 bit��chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
    uint8_t mains_power_shooter_output : 1;  //2 bit��shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
    //uint8_t error;
} ext_game_robot_status_t;

/* ID: 0X0202  Byte: 14    ʵʱ������������ */
typedef __packed struct
{
    uint16_t chassis_volt;                  //���������ѹ ��λ ����
    uint16_t chassis_current;               //����������� ��λ ����
    float chassis_power;                    //����������� ��λ W ��
    uint16_t chassis_power_buffer;          //���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
    uint16_t shooter_id1_17mm_cooling_heat; //1 �� 17mm ǹ������
    uint16_t shooter_id2_17mm_cooling_heat; //2 �� 17mm ǹ������
    uint16_t shooter_id1_42mm_cooling_heat; //42mm ǹ������
    //uint8_t error;
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct
{
    float x;   //λ�� x ���꣬��λ m
    float y;   //λ�� y ���꣬��λ m
    float z;   //λ�� z ���꣬��λ m
    float yaw; //λ��ǹ�ڣ���λ��
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

/* ID: 0x0205  Byte:  3    ���л���������״̬���� */
typedef __packed struct
{
    uint16_t energy_point;
    uint8_t attack_time;
    uint8_t error;
} ext_aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct
{
    uint8_t armor_id : 4;  //bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0
    uint8_t hurt_type : 4; //bit 4-7��Ѫ���仯����
	/*
	0x0 װ���˺���Ѫ��
	0x1 ģ����߿�Ѫ��
	0x2 �����ٿ�Ѫ��
	0x3 ��ǹ��������Ѫ��
	0x4 �����̹��ʿ�Ѫ��
	0x5 װ��ײ����Ѫ
	*/
    uint8_t error;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  6    ʵʱ������� */
typedef __packed struct
{
	uint8_t bullet_type; //�ӵ�����: 1��17mm ���� 2��42mm ����
	uint8_t shooter_id;  //������� ID��
	/*
	1��1 �� 17mm �������
	2��2 �� 17mm �������
	3��42mm �������
	*/
	uint8_t bullet_freq; //�ӵ���Ƶ ��λ Hz
	float bullet_speed;  //�ӵ����� ��λ m/s
    uint8_t error;
} ext_shoot_data_t;

/* ID: 0x0208  Byte:  2    ʵʱ������� */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm; //17mm �ӵ�ʣ�෢����Ŀ
	uint16_t bullet_remaining_num_42mm; //42mm �ӵ�ʣ�෢����Ŀ
	uint16_t coin_remaining_num;        //ʣ��������
    uint8_t error;
} ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  4    ������ RFID ״̬ */
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

/* ����ϵͳ���ݣ��ܽṹ�� */
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


/*float��������*/
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




extern void referee_system_init(void); //��������ʼ��
REFEREE_t *return_referee_point(void); //���غ���ָ��
extern uint8_t referee_read_data(TimerHandle_t xTimer);
uint8_t re_red_or_blue(void);

//���ز���ϵͳ����
/* ID: 0x0102  Byte:  4    ����վ������ʶ */
uint8_t referee_supply_projectile_id(void); //����վ�� ID��
uint8_t referee_supply_robot_id(void); //���������� ID��
uint8_t referee_supply_projectile_step(void); //�����ڿ���״̬��
uint8_t referee_supply_projectile_num(void); //����������
/* ID: 0X0201  Byte: 15    ����������״̬ */
uint8_t referee_robot_id(void); //�������� ID
uint8_t referee_robot_level(void); //�����˵ȼ�
uint16_t referee_remain_HP(void); //������ʣ��Ѫ��
uint16_t referee_max_HP(void); //����������Ѫ��
uint16_t referee_shooter_id1_17mm_cooling_rate(void); //������ 1 �� 17mm ǹ��ÿ����ȴֵ
uint16_t referee_shooter_id1_17mm_cooling_limit(void); //������ 1 �� 17mm ǹ����������
uint16_t referee_shooter_id1_17mm_speed_limit(void); //������ 1 �� 17mm ǹ�������ٶ� ��λ m/s
uint16_t referee_shooter_id2_17mm_cooling_rate(void); //������ 2 �� 17mm ǹ��ÿ����ȴֵ
uint16_t referee_shooter_id2_17mm_cooling_limit(void); //������ 2 �� 17mm ǹ����������
uint16_t referee_shooter_id2_17mm_speed_limit(void); //������ 2 �� 17mm ǹ�������ٶ� ��λ m/s
uint16_t referee_shooter_id1_42mm_cooling_rate(void); //������ 42mm ǹ��ÿ����ȴֵ
uint16_t referee_shooter_id1_42mm_cooling_limit(void); //������ 42mm ǹ����������
uint16_t referee_shooter_id1_42mm_speed_limit(void); //������ 42mm ǹ�������ٶ� ��λ m/s
uint16_t referee_chassis_power_limit(void); //�����˵��̹�����������
/* ID: 0X0202  Byte: 14    ʵʱ������������ */
uint16_t referee_chassis_volt(void); //���������ѹ ��λ ����
uint16_t referee_chassis_current(void); //����������� ��λ ����
float referee_chassis_power(void); //����������� ��λ W ��
uint16_t referee_chassis_power_buffer(void); //���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
uint16_t referee_shooter_id1_17mm_cooling_heat(void); //1 �� 17mm ǹ������
uint16_t referee_shooter_id2_17mm_cooling_heat(void); //2 �� 17mm ǹ������
uint16_t referee_shooter_id1_42mm_cooling_heat(void); //42mm ǹ������
/* ID: 0x0203  Byte: 16    ������λ������ */
float referee_yaw(void); //λ��ǹ�ڣ���λ��
/* ID: 0x0206  Byte:  1    �˺�״̬���� */
uint8_t referee_armor_id(void); //bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0
uint8_t referee_hurt_type(void); //bit 4-7��Ѫ���仯����
/* ID: 0x0207  Byte:  6    ʵʱ������� */
uint8_t bullet_type(void); //�ӵ�����: 1��17mm ���� 2��42mm ����
uint8_t shooter_id(void); //������� ID��
uint8_t bullet_freq(void); //�ӵ���Ƶ ��λ Hz
float bullet_speed(void); //�ӵ����� ��λ m/s
/* ID: 0x0208  Byte:  2    ʵʱ������� */
uint16_t bullet_remaining_num_17mm(void); //17mm �ӵ�ʣ�෢����Ŀ
uint16_t bullet_remaining_num_42mm(void); //42mm �ӵ�ʣ�෢����Ŀ
uint16_t coin_remaining_num(void); //ʣ��������
	
#endif
