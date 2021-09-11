#include "RemoteDeal.h"
#include "MathLib.h"

/*遥控结构体*/
REMOTE_t REMOTE;
/*遥控死区*/
static const u8 DeadZone = 10;


/*************************************************************************************************
*名称:	Get_RemoteDeal_Point
*功能:	返回处理后的遥控数值控制变量，通过指针传递方式传递信息
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
REMOTE_t *Return_RemoteDeal_Point(void)
{
    return &REMOTE;
}

/*************************************************************************************************
*名称:	Remote_Data_Init
*功能:	遥控数值处理初始化
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
void Remote_Data_Init(void)
{
    /*函数映射*/
    REMOTE.Get_Remote_Point = Return_Remote_Point;

    /*遥控数值初始化*/
    REMOTE.RC_ctrl->ch0 = REMOTE.RC_ctrl->ch1 = REMOTE.RC_ctrl->ch2 = REMOTE.RC_ctrl->ch3 = 0;
    REMOTE.RC_ctrl->s1 = REMOTE.RC_ctrl->s2 = REMOTE.RC_ctrl->sw = REMOTE.RC_ctrl->Flag = 0;
    REMOTE.RC_ctrl->KV.x = REMOTE.RC_ctrl->KV.y = REMOTE.RC_ctrl->KV.z = 0;
    REMOTE.RC_ctrl->KV.key = REMOTE.RC_ctrl->KV.press_l = REMOTE.RC_ctrl->KV.press_r = 0;



    /*低通滤波初始化*/
    First_Order_Init(&REMOTE.RC_X, 0.08);
    First_Order_Init(&REMOTE.RC_Y, 0.08);
    First_Order_Init(&REMOTE.RC_Z, 0.08);

    First_Order_Init(&REMOTE.KM_X, 0.005);
    First_Order_Init(&REMOTE.KM_Y, 0.005);
    First_Order_Init(&REMOTE.KM_Z, 0.005);

    /*获取遥控指针*/
    REMOTE.RC_ctrl = REMOTE.Get_Remote_Point();
}



/*************************************************************************************************
*名称:	Rc_Deal
*功能:	处理遥杆拨杆的值
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
static void Rc_Deal(void)
{
    /*死区处理*/
    REMOTE.RC_ctrl->ch0 = Dead_Zone(REMOTE.RC_ctrl->ch0, DeadZone);
    REMOTE.RC_ctrl->ch1 = Dead_Zone(REMOTE.RC_ctrl->ch1, DeadZone);
    REMOTE.RC_ctrl->ch2 = Dead_Zone(REMOTE.RC_ctrl->ch2, DeadZone);

    /*滤波处理*/
    First_Order(&REMOTE.RC_X, REMOTE.RC_ctrl->ch1);
    First_Order(&REMOTE.RC_Y, REMOTE.RC_ctrl->ch0);
    First_Order(&REMOTE.RC_Z, REMOTE.RC_ctrl->ch2);
}



/*************************************************************************************************
*名称:	KEY_MOUSE
*功能:	处理键盘鼠标的值
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
static void Key_Mouse_Deal(void)
{
    uint16_t key = REMOTE.RC_ctrl->KV.key;

    /*前后*/
    if(key & KEY_PRESSED_OFFSET_W)
    {
        REMOTE.RC_ctrl->KV.kv0 += 10;
    }
    else if(key & KEY_PRESSED_OFFSET_S)
    {
        REMOTE.RC_ctrl->KV.kv0 -= 10;
    }
    else
    {
        REMOTE.RC_ctrl->KV.kv0 = 0;
    }

    /*左右*/
    if(key & KEY_PRESSED_OFFSET_A)
    {
        REMOTE.RC_ctrl->KV.kv1 -= 10;
    }
    else if(key & KEY_PRESSED_OFFSET_D)
    {
        REMOTE.RC_ctrl->KV.kv1 += 10;
    }
    else
    {
        REMOTE.RC_ctrl->KV.kv1 = 0;
    }

    /*Y轴旋转*/
    REMOTE.RC_ctrl->KV.kv3 = (float)REMOTE.RC_ctrl->KV.x * 15;

    /*Z轴旋转*/
    REMOTE.RC_ctrl->KV.kv2 = (float)REMOTE.RC_ctrl->KV.y * 15;

    /*限制幅度处理*/
    REMOTE.RC_ctrl->KV.kv0 = limit(REMOTE.RC_ctrl->KV.kv0, 660, -660);
    REMOTE.RC_ctrl->KV.kv1 = limit(REMOTE.RC_ctrl->KV.kv1, 660, -660);
    REMOTE.RC_ctrl->KV.kv2 = limit(REMOTE.RC_ctrl->KV.kv2, 660, -660);
    REMOTE.RC_ctrl->KV.kv3 = limit(REMOTE.RC_ctrl->KV.kv3, 660, -660);

    /*滤波处理*/
    First_Order(&REMOTE.KM_X, REMOTE.RC_ctrl->KV.kv1);
    First_Order(&REMOTE.KM_Y, REMOTE.RC_ctrl->KV.kv0);
    First_Order(&REMOTE.KM_Z, REMOTE.RC_ctrl->KV.kv2);

}



/*************************************************************************************************
*名称:	RC_DATA_DEAL
*功能:	遥控数值处理
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
void Remote_Data_Deal(void)
{
    /*数据更新标志位*/
    if(REMOTE.RC_ctrl->Flag == 1)
    {
        Rc_Deal();
    }
}









