#include "ui_task.h"
#include "struct_typedef.h"
#include "rm_cilent_ui.h"
#include "string.h"
#include "chassis_behaviour.h"
#include "referee_deal.h"
#include "can_2_receive.h"
#include "shoot_task.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//#pragma diag_suppress 177


static void ui_init(void);           //ui��ʼ��
static void print_mode(void);        //����ģʽ��ʾ
static void print_cap_voltage(void); //��ѹ��ʾ
static void print_shoot(void);       //���ģʽ��ʾ
static void print_hurt(void);        //����״̬��ʾ
static void print_pitch(void);       //pitch�Ƕ���ʾ
Graph_Data G1, G2, G3, G4, G5, G6, G7, G8, G9, G10, G11;

String_Data CH_SHOOT;
String_Data CH_FLRB;
String_Data CH_MODE;
Float_Data CH_POWER;
String_Data CH_MODE_G;
String_Data CH_HURT;
Float_Data CH_ID;
Float_Data CH_PITCH;
Float_Data CAP_VOLTAGE;


const static char *fire_mode_ui[6] = {
    "FIRE_H",          //����Ƶ
	"FIRE_L",          //����Ƶ
    "AUTO_FIRE",       //�Զ�����
    "STOP_FIRE",       //ֹͣ����
    "BACK",            //�˵�
	"FIRE_ERROR"};     //����
const static char *chassis_mode_ui[6] = {
    "ZERO_FORCE",      //��������
    "FOLLOW",          //����
    "NO_FOLLOW",       //������
    "TWIST_WAIST",     //Ť��
    "ROTATION",        //С����
    "BATTERY"};        //��̨ģʽ
const static char *gimbal_mode_ui[6] = {
	"ZERO_FORCE",      //��̨����
	"MANUAL",          //�ֶ�״̬
	"AUTOATTACK",      //����״̬
	"AUTOBUFF",        //���״̬
	"REPLENISHMEN",    //����״̬
	"DOUBLE_GIMBAL"};  //˫��̨״̬������̨���飬����̨������ 


void ui_task(void *pvParameters)
{
    static Uint8_t i;

    ui_init();
    while (1)
    {
        i++;
        //����ģʽ
        print_mode();
        //�������ݵ�ѹ��ʾ
        print_cap_voltage();
        //���״̬��ģʽ��ʾ
        print_shoot();
        //pitch�Ƕ���ʾ
        print_pitch();
        //��ʾ����״̬(��������ݸ��¹���)
        if (i % 10 == 0)
            print_hurt();
        //Ƶ�ʿ���
        vTaskDelay(10);
    }
}

//����ģʽ��ʾ
static void print_mode(void)
{
    CH_MODE.Graph_Control.width = strlen(chassis_mode_ui[re_chassis_behaviour()]);
    strcpy(CH_MODE.show_Data, chassis_mode_ui[re_chassis_behaviour()]);
    CH_MODE.Graph_Control.operate_tpye = UI_Graph_Change;
    My_Char_Refresh(CH_MODE);
}

//���ݵ�ѹ��ʾ
static void print_cap_voltage(void)
{
    CAP_VOLTAGE.graph_Float = re_capacitance_voltage();
    CAP_VOLTAGE.operate_tpye = UI_Graph_Change;
    My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);
}


//���ģʽ��ʾ
static void print_shoot(void)
{
    if (re_shoot_status() == FIRE_H || re_shoot_status() == FIRE_L || re_shoot_status() == AUTO_FIRE)
        CH_SHOOT.Graph_Control.color = UI_Color_Purplish_red; //�����ڴ�,����ɫ���
	else if (re_shoot_status() == BACK)
		CH_SHOOT.Graph_Control.color = UI_Color_Green; //�������˵�,����ɫ����ɫ
    else if (re_shoot_status() == STOP_FIRE)
        CH_SHOOT.Graph_Control.color = UI_Color_Yellow; //Ĭ��Ϊ��ɫ

	strcpy(CH_SHOOT.show_Data, fire_mode_ui[re_shoot_status()]);

    CH_SHOOT.Graph_Control.operate_tpye = UI_Graph_Change; //ģʽΪ�޸�UI
    My_Char_Refresh(CH_SHOOT);                             // ˢ��UI
}

static void print_hurt(void)
{
    static u16 HP[2];
    HP[1] = HP[0];
    HP[0] = referee_remain_HP();
    int8_t hurt = HP[1] - HP[0];

    if (hurt == 2) //ײ��
        Char_Draw(&CH_HURT, "086", UI_Graph_Change, 4, UI_Color_Purplish_red, 30, 5, 80, 700, "STRIKE");
    else if (hurt >= 10 && hurt <= 100) //С�����˺�
        Char_Draw(&CH_HURT, "086", UI_Graph_Change, 4, UI_Color_Purplish_red, 30, 5, 80, 700, "17mm_ATTACKING");
    else if (hurt == 100) //�����˺�
        Char_Draw(&CH_HURT, "086", UI_Graph_Change, 4, UI_Color_Purplish_red, 30, 5, 80, 700, "42mm_ATTACKING");
    else //��ȫ
        Char_Draw(&CH_HURT, "086", UI_Graph_Change, 4, UI_Color_Yellow, 30, 5, 80, 700, "SAFE");

    My_Char_Refresh(CH_HURT);
}

static void print_pitch(void)
{
    CH_PITCH.operate_tpye = UI_Graph_Change;
    CH_PITCH.graph_Float = re_gimbal_pitch_angle();
    My_Graph_Refresh((Graph_Data *)&CH_PITCH);
}

static void ui_init(void)
{
    String_Data voltage, pitch;

    //����ui
    Line_Draw(&G1, "091", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 960, 330, 960, 620);
    Line_Draw(&G2, "092", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 880, 580, 1040, 580);
    Line_Draw(&G3, "093", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 800, 540, 1120, 540);
    Line_Draw(&G4, "094", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 880, 500, 1040, 500);
    Line_Draw(&G5, "095", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 900, 420, 1020, 420);
    Line_Draw(&G6, "096", UI_Graph_ADD, 4, UI_Color_Purplish_red, 1, 920, 370, 1000, 370);

    My_Graph_Refresh((Graph_Data *)&G1);
    My_Graph_Refresh((Graph_Data *)&G2);
    My_Graph_Refresh((Graph_Data *)&G3);
    My_Graph_Refresh((Graph_Data *)&G4);
    My_Graph_Refresh((Graph_Data *)&G5);
    My_Graph_Refresh((Graph_Data *)&G6);

    //�ܻ�ui����
    Char_Draw(&CH_HURT, "086", UI_Graph_ADD, 4, UI_Color_Yellow, 24, 5, 80, 700, "SAFE");
    My_Char_Refresh(CH_HURT);
    //���ģʽui����
    Char_Draw(&CH_SHOOT, "087", UI_Graph_ADD, 4, UI_Color_Yellow, 24, 5, 80, 780, "SEMI");
    My_Char_Refresh(CH_SHOOT);
    //����ģʽui����
    Char_Draw(&CH_MODE, "093", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 8, 80, 880, chassis_mode_ui[0]);
    My_Char_Refresh(CH_MODE);
    //����ui����
    Float_Draw(&CAP_VOLTAGE, "075", UI_Graph_ADD, 6, UI_Color_Yellow, 20, 32, 3, 1250, 550, re_capacitance_voltage());
    My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);
    Char_Draw(&voltage, "078", UI_Graph_ADD, 8, UI_Color_Yellow, 20, 5, 1080, 550, "VOLTAGE: ");
    My_Char_Refresh(voltage);
    //pitch��ui����
    Float_Draw(&CH_PITCH, "076", UI_Graph_ADD, 6, UI_Color_Yellow, 20, 32, 3, 1250, 480, re_gimbal_pitch_angle());
    My_Graph_Refresh((Graph_Data *)&CH_PITCH);
    Char_Draw(&pitch, "079", UI_Graph_ADD, 8, UI_Color_Yellow, 20, 5, 1080, 480, "PITCH:   ");
    My_Char_Refresh(pitch);
}

