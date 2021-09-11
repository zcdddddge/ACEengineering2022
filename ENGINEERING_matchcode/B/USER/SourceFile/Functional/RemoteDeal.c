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
*名称:	Remote_Data_Zero
*功能:	遥控数值清零
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
static void Remote_Data_Zero(void)
{
    /*遥控数值初始化*/
    REMOTE.RC_ctrl->ch0 = REMOTE.RC_ctrl->ch1 = REMOTE.RC_ctrl->ch2 = REMOTE.RC_ctrl->ch3 = 0;
    REMOTE.RC_ctrl->sw = REMOTE.RC_ctrl->Flag = 0;
    REMOTE.RC_ctrl->KV.x = REMOTE.RC_ctrl->KV.y = REMOTE.RC_ctrl->KV.z = 0;
    REMOTE.RC_ctrl->KV.key = REMOTE.RC_ctrl->KV.press_l = REMOTE.RC_ctrl->KV.press_r = 0;

    /*状态值初始化 全部暂停*/
				REMOTE.state.Grasp_Up = 0; //
				REMOTE.state.Translation = 1;  //
				REMOTE.state.Telescoping = 0;
				REMOTE.state.Clap = 0;
				REMOTE.state.Flip = 0;
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
    /*遥控数值清零*/
    Remote_Data_Zero();
    REMOTE.RC_ctrl->s1 = REMOTE.RC_ctrl->s2 = 2; //此键位为断电键位

    /*低通滤波初始化*/
    First_Order_Init(&REMOTE.RC_X, 0.08);
    First_Order_Init(&REMOTE.RC_Y, 0.08);
    First_Order_Init(&REMOTE.RC_Z, 0.08);

    First_Order_Init(&REMOTE.KM_X, 0.08);
    First_Order_Init(&REMOTE.KM_Y, 0.08);
    First_Order_Init(&REMOTE.KM_Z, 0.08);

    /*获取遥控指针*/
    REMOTE.RC_ctrl = Return_Remote_Point();
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
*说明:	分支结构可以优化
*************************************************************************************************/
static void Key_Mouse_Deal(void)
{
    uint16_t key = REMOTE.RC_ctrl->KV.key;

    /*************************************底盘前后*********************************/
    if (key & KEY_PRESSED_OFFSET_W)
    {
        if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_W)
        {
            REMOTE.RC_ctrl->KV.kv0 = 6000;
        }
        else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_W)
        {
            REMOTE.RC_ctrl->KV.kv0 = 200;
        }
        else
        {
            REMOTE.RC_ctrl->KV.kv0 = 3000;
        }
    }
    else if (key & KEY_PRESSED_OFFSET_S)
    {
        if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_S)
        {
            REMOTE.RC_ctrl->KV.kv0 = -6000;
        }
        else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_S)
        {
            REMOTE.RC_ctrl->KV.kv0 = -200;
        }
        else
        {
            REMOTE.RC_ctrl->KV.kv0 = -3000;
        }
    }
    else
    {
        REMOTE.RC_ctrl->KV.kv0 = 0;
    }

    /**************************************底盘左右*********************************/
    if (key & KEY_PRESSED_OFFSET_A)
    {
        if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_A)
        {
            REMOTE.RC_ctrl->KV.kv1 = -6000;
        }
        else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_A)
        {
            REMOTE.RC_ctrl->KV.kv1 = -200;
        }
        else
        {
            REMOTE.RC_ctrl->KV.kv1 = -3000;
        }
    }
    else if (key & KEY_PRESSED_OFFSET_D)
    {
        if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_D)
        {
            REMOTE.RC_ctrl->KV.kv1 = 6000;
        }
        else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_D)
        {
            REMOTE.RC_ctrl->KV.kv1 = 200;
        }
        else
        {
            REMOTE.RC_ctrl->KV.kv1 = 3000;
        }
    }
    else
    {
        REMOTE.RC_ctrl->KV.kv1 = 0;
    }

    /*************************************鼠标X轴************************************/
    REMOTE.RC_ctrl->KV.kv3 = (float)REMOTE.RC_ctrl->KV.x * 5.0f;
    /**************************************鼠标Y轴***********************************/
    REMOTE.RC_ctrl->KV.kv2 = (float)REMOTE.RC_ctrl->KV.y * 5.0f;

    /*******************************自动夹取********************************/
    if (key & KEY_PRESSED_OFFSET_Q)
    {
            REMOTE.state.Grasp_Up = 1; // 自动夹取
    }

    /********************************兑换抬升*******************************/
    if (key & KEY_PRESSED_OFFSET_E)
    {
            REMOTE.state.Grasp_Up = 2; // 自动夹取
    }

    /********************************翻转复位*******************************/
    if (key & KEY_PRESSED_OFFSET_R)
    {
            REMOTE.state.Grasp_Up = 3; // 自动夹取
    }

    /*********************************RFID********************************/
    if (key & KEY_PRESSED_OFFSET_F)
    {

        if (key & KEY_PRESSED_OFFSET_F && key & KEY_PRESSED_OFFSET_CTRL)
        {
            REMOTE.state.Gimbal_Yaw = 1;
        }
        else
        {
            REMOTE.state.Gimbal_Yaw = 0;
        }
    }

    /*********************************救援爪********************************/
    if (key & KEY_PRESSED_OFFSET_G)
    {

        if (key & KEY_PRESSED_OFFSET_G && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.RFID = 0; //RFID缩
            REMOTE.state.Barrier = 0; //救援爪缩
        }
        else
        {
						REMOTE.state.RFID = 1; //RFID伸
            REMOTE.state.Barrier = 1; //救援爪伸
        }
    }

    /* ========================================以下为底盘====================================== */

    /*********************************正常**********************************/
    if (key & KEY_PRESSED_OFFSET_SHIFT)
    {

        REMOTE.state.Independent = 1; //底盘正常运动
        REMOTE.state.Rotation = 2;    //小陀螺模式暂停
        REMOTE.state.Wiggle = 2;      //扭腰模式暂停
    }

    /*********************************大陀螺**********************************/
    if (key & KEY_PRESSED_OFFSET_Z)
    {

        REMOTE.state.Independent = 2; //底盘正常暂停
        REMOTE.state.Rotation = 1;    //大陀螺模式
        REMOTE.state.Wiggle = 2;      //扭腰模式暂停
    }
		
    /* ======================================以上为底盘===================================== */
    /**********************************松夹**********************************/
    if (key & KEY_PRESSED_OFFSET_X)
    {
        if (key & KEY_PRESSED_OFFSET_X && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.Translation = 0;  //上装复位关闭
        }
        else
        {
						REMOTE.state.Translation = 1;  //上装复位关闭
				}
    }

    /*******************************兑换一箱********************************/
    if (key & KEY_PRESSED_OFFSET_C)
    {
        if (key & KEY_PRESSED_OFFSET_C && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.Telescoping = 0;  //上装复位关闭
        }
        else
        {
						REMOTE.state.Telescoping = 1;  //上装复位关闭
				}
    }

    /********************************兑换二箱********************************/
    if (key & KEY_PRESSED_OFFSET_V)
    {
        if (key & KEY_PRESSED_OFFSET_V && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.Clap = 0;  //上装复位关闭
        }
        else
        {
						REMOTE.state.Clap = 1;  //上装复位关闭
				}
    }

    /********************************兑换三箱********************************/
    if (key & KEY_PRESSED_OFFSET_B)
    {
        if (key & KEY_PRESSED_OFFSET_B && key & KEY_PRESSED_OFFSET_CTRL)
        {
						REMOTE.state.Flip = 0;  //上装复位关闭
        }
        else
        {
						REMOTE.state.Flip = 1;  //上装复位关闭
				}
    }

    /*限制幅度处理*/
    REMOTE.RC_ctrl->KV.kv0 = limit(REMOTE.RC_ctrl->KV.kv0, 660, -660);
    REMOTE.RC_ctrl->KV.kv1 = limit(REMOTE.RC_ctrl->KV.kv1, 660, -660);
    REMOTE.RC_ctrl->KV.kv2 = limit(REMOTE.RC_ctrl->KV.kv2, 660, -660);
    REMOTE.RC_ctrl->KV.kv3 = limit(REMOTE.RC_ctrl->KV.kv3, 660, -660);
    /*滤波处理*/
    First_Order(&REMOTE.KM_X, REMOTE.RC_ctrl->KV.kv0); //底盘左右
    First_Order(&REMOTE.KM_Y, REMOTE.RC_ctrl->KV.kv1); //底盘前后
    First_Order(&REMOTE.KM_Z, REMOTE.RC_ctrl->KV.kv3); // 鼠标x轴
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
    static unsigned char _s1 = 0;

    if (_s1 != REMOTE.RC_ctrl->s1)
        Remote_Data_Zero();

    if (REMOTE.RC_ctrl->Flag)
    {
        if (REMOTE.RC_ctrl->s1 == 3)
        {
            Key_Mouse_Deal();
        }
        else
        {
            Rc_Deal();
        }

        REMOTE.RC_ctrl->Flag = 0;
    }

    _s1 = REMOTE.RC_ctrl->s1;
}
