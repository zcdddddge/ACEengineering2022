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


static void ui_init(void);           //ui初始化
static void print_mode(void);        //底盘模式显示
static void print_cap_voltage(void); //电压显示
static void print_shoot(void);       //射击模式显示
static void print_hurt(void);        //受伤状态显示
static void print_pitch(void);       //pitch角度显示
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
    "FIRE_H",          //高射频
	"FIRE_L",          //低射频
    "AUTO_FIRE",       //自动发射
    "STOP_FIRE",       //停止发射
    "BACK",            //退弹
	"FIRE_ERROR"};     //错误
const static char *chassis_mode_ui[6] = {
    "ZERO_FORCE",      //底盘无力
    "FOLLOW",          //跟随
    "NO_FOLLOW",       //不跟随
    "TWIST_WAIST",     //扭腰
    "ROTATION",        //小陀螺
    "BATTERY"};        //炮台模式
const static char *gimbal_mode_ui[6] = {
	"ZERO_FORCE",      //云台无力
	"MANUAL",          //手动状态
	"AUTOATTACK",      //自瞄状态
	"AUTOBUFF",        //打符状态
	"REPLENISHMEN",    //补给状态
	"DOUBLE_GIMBAL"};  //双云台状态（主云台自瞄，副云台操作） 


void ui_task(void *pvParameters)
{
    static Uint8_t i;

    ui_init();
    while (1)
    {
        i++;
        //底盘模式
        print_mode();
        //超级电容电压显示
        print_cap_voltage();
        //射击状态和模式显示
        print_shoot();
        //pitch角度显示
        print_pitch();
        //显示受伤状态(避免该数据更新过快)
        if (i % 10 == 0)
            print_hurt();
        //频率控制
        vTaskDelay(10);
    }
}

//底盘模式显示
static void print_mode(void)
{
    CH_MODE.Graph_Control.width = strlen(chassis_mode_ui[re_chassis_behaviour()]);
    strcpy(CH_MODE.show_Data, chassis_mode_ui[re_chassis_behaviour()]);
    CH_MODE.Graph_Control.operate_tpye = UI_Graph_Change;
    My_Char_Refresh(CH_MODE);
}

//电容电压显示
static void print_cap_voltage(void)
{
    CAP_VOLTAGE.graph_Float = re_capacitance_voltage();
    CAP_VOLTAGE.operate_tpye = UI_Graph_Change;
    My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);
}


//射击模式显示
static void print_shoot(void)
{
    if (re_shoot_status() == FIRE_H || re_shoot_status() == FIRE_L || re_shoot_status() == AUTO_FIRE)
        CH_SHOOT.Graph_Control.color = UI_Color_Purplish_red; //若正在打弹,则颜色变红
	else if (re_shoot_status() == BACK)
		CH_SHOOT.Graph_Control.color = UI_Color_Green; //若正在退弹,则颜色变绿色
    else if (re_shoot_status() == STOP_FIRE)
        CH_SHOOT.Graph_Control.color = UI_Color_Yellow; //默认为黄色

	strcpy(CH_SHOOT.show_Data, fire_mode_ui[re_shoot_status()]);

    CH_SHOOT.Graph_Control.operate_tpye = UI_Graph_Change; //模式为修改UI
    My_Char_Refresh(CH_SHOOT);                             // 刷新UI
}

static void print_hurt(void)
{
    static u16 HP[2];
    HP[1] = HP[0];
    HP[0] = referee_remain_HP();
    int8_t hurt = HP[1] - HP[0];

    if (hurt == 2) //撞击
        Char_Draw(&CH_HURT, "086", UI_Graph_Change, 4, UI_Color_Purplish_red, 30, 5, 80, 700, "STRIKE");
    else if (hurt >= 10 && hurt <= 100) //小弹丸伤害
        Char_Draw(&CH_HURT, "086", UI_Graph_Change, 4, UI_Color_Purplish_red, 30, 5, 80, 700, "17mm_ATTACKING");
    else if (hurt == 100) //大弹丸伤害
        Char_Draw(&CH_HURT, "086", UI_Graph_Change, 4, UI_Color_Purplish_red, 30, 5, 80, 700, "42mm_ATTACKING");
    else //安全
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

    //吊射ui
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

    //受击ui加入
    Char_Draw(&CH_HURT, "086", UI_Graph_ADD, 4, UI_Color_Yellow, 24, 5, 80, 700, "SAFE");
    My_Char_Refresh(CH_HURT);
    //射击模式ui加入
    Char_Draw(&CH_SHOOT, "087", UI_Graph_ADD, 4, UI_Color_Yellow, 24, 5, 80, 780, "SEMI");
    My_Char_Refresh(CH_SHOOT);
    //底盘模式ui加入
    Char_Draw(&CH_MODE, "093", UI_Graph_ADD, 8, UI_Color_Yellow, 25, 8, 80, 880, chassis_mode_ui[0]);
    My_Char_Refresh(CH_MODE);
    //电容ui加入
    Float_Draw(&CAP_VOLTAGE, "075", UI_Graph_ADD, 6, UI_Color_Yellow, 20, 32, 3, 1250, 550, re_capacitance_voltage());
    My_Graph_Refresh((Graph_Data *)&CAP_VOLTAGE);
    Char_Draw(&voltage, "078", UI_Graph_ADD, 8, UI_Color_Yellow, 20, 5, 1080, 550, "VOLTAGE: ");
    My_Char_Refresh(voltage);
    //pitch轴ui加入
    Float_Draw(&CH_PITCH, "076", UI_Graph_ADD, 6, UI_Color_Yellow, 20, 32, 3, 1250, 480, re_gimbal_pitch_angle());
    My_Graph_Refresh((Graph_Data *)&CH_PITCH);
    Char_Draw(&pitch, "079", UI_Graph_ADD, 8, UI_Color_Yellow, 20, 5, 1080, 480, "PITCH:   ");
    My_Char_Refresh(pitch);
}

