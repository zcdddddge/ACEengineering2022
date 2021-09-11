#include "ChassisMotor.h"
#include "MathLib.h"

/*底盘轮子速度环参数*/
#define WHEEL_MOTOR1_P   	4.0f
#define WHEEL_MOTOR1_I   	0.0f
#define WHEEL_MOTOR1_D   0.02f
#define WHEEL_MOTOR2_P  	4.0f
#define	WHEEL_MOTOR2_I   	0.0f
#define WHEEL_MOTOR2_D   0.02f
#define WHEEL_MOTOR3_P   	4.0f
#define WHEEL_MOTOR3_I   	0.0f
#define WHEEL_MOTOR3_D   0.02f
#define WHEEL_MOTOR4_P   	4.0f
#define WHEEL_MOTOR4_I   	0.0f
#define WHEEL_MOTOR4_D   0.02f
/*底盘救援电机参数*/
/*救援*/
#define  RESCUE_S_P   0.0f
#define  RESCUE_S_I   0.0f
#define  RESCUE_S_D   0.0f
#define  RESCUE_P_P   0.0f
#define  RESCUE_P_I   0.0f
#define  RESCUE_P_D   0.0f


/*************************************************************************************************
*名称:	Wheel_Motor_Init
*功能:	底盘轮子电机初始化
*形参: 	C_t *C
*返回:	无
*说明:	无
*************************************************************************************************/
void Wheel_Motor_Init(C_t *C)
{
    u8 CMi = 0;
    float Spid[4][3] =
    {
        {WHEEL_MOTOR1_P, WHEEL_MOTOR1_I, WHEEL_MOTOR1_D},
        {WHEEL_MOTOR2_P, WHEEL_MOTOR2_I, WHEEL_MOTOR2_D},
        {WHEEL_MOTOR3_P, WHEEL_MOTOR3_I, WHEEL_MOTOR3_D},
        {WHEEL_MOTOR4_P, WHEEL_MOTOR4_I, WHEEL_MOTOR4_D},
    };

    /*函数映射*/
    C->Can_Send_Rescue 		= CAN1_205_To_208_SEND;
    C->Can_Send_Wheel	 		= CAN1_201_To_204_SEND;
    C->Get_Rescue_Data		=	Return_Can1_205_208_Data;
    C->Get_Wheel_Data			=	Return_Can1_201_204_Data;
    C->Get_Rescue_Encoder	=	Return_Can1_205_Encoder;

    /*清零处理*/
    MotorValZero(&C->WheelMotor[0]);
    MotorValZero(&C->WheelMotor[1]);
    MotorValZero(&C->WheelMotor[2]);
    MotorValZero(&C->WheelMotor[3]);

    for(CMi = 0; CMi < 4; CMi ++)
    {
        C->WheelMotor[CMi].ID = CMi + 1;
        PID_INIT(&C->WheelMotor[CMi].SPID, Spid[CMi][0], Spid[CMi][1], Spid[CMi][2], 15000, 16000);	//速度环初始化
        C->WheelMotor[CMi].MotorType = CHASSIS_M;																								//初始化电机种类
        C->WheelMotor[CMi].Radio = 19;																													//初始化底盘电机减速比
    }

    C->WheelData = C->Get_Wheel_Data();																													//获取底盘电机数据指针
}


/*************************************************************************************************
*名称:	Get_Chassis_Data
*功能:	获取底盘电机数据
*形参: 	C_t *C
*返回:	无
*说明:	Speed1,Speed2,Speed3,Speed4为测试数据
*************************************************************************************************/
#if 0
    int16_t Speed1, Speed2, Speed3, Speed4;
#endif
void Get_Chassis_Data(C_t *C)
{
    C->WheelMotor[0].Speed = C->WheelData[1];
    C->WheelMotor[1].Speed = C->WheelData[3];
    C->WheelMotor[2].Speed = C->WheelData[5];
    C->WheelMotor[3].Speed = C->WheelData[7];
    C->RescueMotor.Speed	 = C->RescueData[1];
    #if 0
    Speed1 = MOTOR_C[0].Speed;
    Speed2 = MOTOR_C[1].Speed;
    Speed3 = MOTOR_C[2].Speed;
    Speed4 = MOTOR_C[3].Speed;
    #endif
}



/*************************************************************************************************
*名称:	CHASSIS_TEXT_DRIVE
*功能:	驱动底盘电机测试模式
*形参: 	C_t *C
*返回:	无
*说明:	无
*************************************************************************************************/
void Chassis_Text_Drive(C_t *C)
{

}



/*************************************************************************************************
*名称:	CHASSIS_FOLLOW_DRIVE
*功能:	底盘跟随模式
*形参: 	C_t *C,float X_IN,float Y_IN,float Z_IN
*返回:	无
*说明:	无
*************************************************************************************************/
void Chassis_Follow_Drive(C_t *C, float X_IN, float Y_IN, float Z_IN)
{
    u8 ID_C = 0;
    int16_t Val[4] = {0, 0, 0, 0};
    int16_t MAX = 0;
    float SPID_OUT[4];

    /*底盘远动分解*/
    C->WheelMotor[0].ExpSpeed = (X_IN + Y_IN + Z_IN);
    C->WheelMotor[1].ExpSpeed = -(X_IN - Y_IN - Z_IN);
    C->WheelMotor[2].ExpSpeed = (X_IN - Y_IN + Z_IN);
    C->WheelMotor[3].ExpSpeed = -(X_IN + Y_IN - Z_IN);

    /*速度增益*/
    C->WheelMotor[0].ExpSpeed *= 12;
    C->WheelMotor[1].ExpSpeed *= 12;
    C->WheelMotor[2].ExpSpeed *= 12;
    C->WheelMotor[3].ExpSpeed *= 12;

    /*PID处理*/
    for(ID_C = 0; ID_C < 4; ID_C ++)
    {
        SPID_OUT[ID_C] = PID_DEAL(&C->WheelMotor[ID_C].SPID, C->WheelMotor[ID_C].ExpSpeed, C->WheelMotor[ID_C].Speed);		//得到输出量
        Val[ID_C] = C->WheelMotor[ID_C].Speed;																														    						//记录电机实时速度值
    }

    /*底盘电机运动失真处理，处理的前提是将PID输出值与速度值认为有线性关系*/
    MAX = RETURN_MAX(Val, 4);									//获取最大速度

    if(MAX == 0 || MAX <= 8750)	 						//最大速度为0或小于电机最大速度，正常赋值
    {
        C->WheelMotor[0].SPID.Out = SPID_OUT[0];
        C->WheelMotor[1].SPID.Out = SPID_OUT[1];
        C->WheelMotor[2].SPID.Out = SPID_OUT[2];
        C->WheelMotor[3].SPID.Out = SPID_OUT[3];
    }
    else																		 //否则进行运动失真处理
    {
        /*确保速度不为0时，才进行处理*/
        if(Val[0] != 0)
            C->WheelMotor[0].SPID.Out = SPID_OUT[0] * int16_t_abs(Val[0]) / MAX;
        else
            C->WheelMotor[0].SPID.Out = SPID_OUT[0];

        if(Val[1] != 0)
            C->WheelMotor[1].SPID.Out = SPID_OUT[1] * int16_t_abs(Val[1]) / MAX;
        else
            C->WheelMotor[1].SPID.Out = SPID_OUT[1];

        if(Val[2] != 0)
            C->WheelMotor[2].SPID.Out = SPID_OUT[2] * int16_t_abs(Val[2]) / MAX;
        else
            C->WheelMotor[2].SPID.Out = SPID_OUT[2];

        if(Val[3] != 0)
            C->WheelMotor[3].SPID.Out = SPID_OUT[3] * int16_t_abs(Val[3]) / MAX;
        else
            C->WheelMotor[3].SPID.Out = SPID_OUT[3];
    }

    /*CAN1通信发送执行*/
    C->Can_Send_Wheel(C->WheelMotor[0].SPID.Out, C->WheelMotor[1].SPID.Out, C->WheelMotor[2].SPID.Out, C->WheelMotor[3].SPID.Out);
}



/*************************************************************************************************
*名称:	CHASSIS_POWEROFF
*功能:	底盘断电
*形参: 	C_t *C
*返回:	无
*说明:	无
*************************************************************************************************/
void Chassis_Poweroff_Drive(C_t *C)
{
    C->WheelMotor[0].SPID.Out = C->WheelMotor[1].SPID.Out = C->WheelMotor[2].SPID.Out = C->WheelMotor[3].SPID.Out = 0;
    C->Can_Send_Wheel(0, 0, 0, 0);
}



/*************************************************************************************************
*名称:	Rescue_Motor_Init
*功能:	救援电机初始化
*形参: 	C_t *C
*返回:	无
*说明:	无
*************************************************************************************************/
void Rescue_Motor_Init(C_t *C)
{
    MotorValZero(&C->RescueMotor);
    C->RescueMotor.ID = 5;																//ID赋值

    C->RescueMotor.Encoder = C->Get_Rescue_Encoder();			//获取救援电机码盘结构体
    C->RescueData = C->Get_Rescue_Data();									//获取救援电机数据指针

    PID_INIT(&C->RescueMotor.PPID, RESCUE_P_P, RESCUE_P_I, RESCUE_P_D, 0.0f, 0.0f);																				//外环初始化
    PID_INIT(&C->RescueMotor.SPID, RESCUE_S_P, RESCUE_S_I, RESCUE_S_D, 0.0f, 0.0f);    																		//内环初始化

    C->RescueMotor.ExpRadian						= C->RescueMotor.Encoder->Radian;					//初始化期望角度
    C->RescueMotor.Encoder->Init_Radian = C->RescueMotor.Encoder->Radian;    			//初始码盘值赋值
    C->RescueMotor.Encoder->Lock_Radian = C->RescueMotor.Encoder->Radian;	    		//初始化上锁角度

    C->RescueMotor.MotorType = RESCUE_M;																					//初始化电机种类
    C->RescueMotor.Radio = 36;																										//初始化底盘电机减速比
}


/*************************************************************************************************
*名称:	Rescue_Ctrl
*功能:	救援电机驱动
*形参: 	C_t *C,int8_t choice
*返回:	无
*说明:	无
*************************************************************************************************/
void Rescue_Ctrl(C_t *C, int8_t choice)
{
    if(choice)
    {


    }
    else
    {

    }

    C->Can_Send_Rescue(C->RescueMotor.SPID.Out, 0, 0, 0);
}
