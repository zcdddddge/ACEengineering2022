#include "GimbalMotor.h"


/*Pitch*/
#define PITCH_P_P  1.0f
#define PITCH_P_I  0.0f
#define PITCH_P_D  0.0f
#define PITCH_S_P  4.0f
#define PITCH_S_I  0.0f
#define PITCH_S_D  0.0f
/*Yaw*/
#define YAW_P_P    401.0f
#define YAW_P_I	 	 0.0f
#define YAW_P_D    0.0f
#define YAW_S_P    3.0f
#define YAW_S_I	 	 0.0f
#define YAW_S_D	 	 0.01f
/*����*/
#define  AMMUNITI_P_P  0.0f
#define  AMMUNITI_P_I  0.0f
#define  AMMUNITI_P_D  0.0f
#define  AMMUNITI_S_P  0.0f
#define  AMMUNITI_S_I  0.0f
#define  AMMUNITI_S_D  0.0f


/*************************************************************************************************
*����:	PY_Motor_Init
*����:	��̨PY������ʼ��
*�β�: 	G_t *G
*����:	��
*˵��:	��
*************************************************************************************************/
void PY_Motor_Init(G_t *G)
{
    /*����ӳ��*/
    G->Can_Send_Gimbal		=	CAN2_205_To_208_SEND;
    G->Get_Gimbal_Data		=	Return_Can2_205_208_Data;
    G->Get_Pitch_Encoder	=	Return_Can2_205_Encoder;
    G->Get_Yaw_Encoder		=	Return_Can2_206_Encoder;

    /*���㴦��*/
    MotorValZero(&G->PitchMotor);
    MotorValZero(&G->YawMotor);

    /*���̸�ֵ*/
    G->PitchMotor.Encoder = G->Get_Pitch_Encoder();
    G->YawMotor.Encoder		=	G->Get_Yaw_Encoder();

    /*PITCH*/
    G->PitchMotor.ID = 6;
    PID_INIT(&G->PitchMotor.PPID, PITCH_P_P, PITCH_P_I, PITCH_P_D, 0.0f, 0.0f);		//�⻷������ʼ��
    PID_INIT(&G->PitchMotor.SPID, PITCH_S_P, PITCH_S_I, PITCH_S_D, 0.0f, 0.0f);		//�ڻ�������ʼ��
    G->PitchMotor.ExpRadian							= G->PitchMotor.Encoder->Radian;					//��ʼ�������Ƕ�
    G->PitchMotor.Encoder->Init_Radian 	= G->PitchMotor.Encoder->Radian;    			//��ʼ����ֵ��ֵ
    G->PitchMotor.Encoder->Lock_Radian 	= G->PitchMotor.Encoder->Radian;	    		//��ʼ�������Ƕ�
    G->PitchMotor.MotorType = PITCH_M;																					//��ʼ���������
    G->PitchMotor.Radio = 19;																										//��ʼ�����̵�����ٱ�

    /*YAW*/
    G->YawMotor.ID = 7;
    PID_INIT(&G->YawMotor.PPID, YAW_P_P, YAW_P_I, YAW_P_D, 0.0f, 0.0f);		//�⻷������ʼ��
    PID_INIT(&G->YawMotor.SPID, YAW_S_P, YAW_S_I, YAW_S_D, 0.0f, 0.0f);		//�ڻ�������ʼ��
    G->YawMotor.ExpRadian							= G->YawMotor.Encoder->Radian;					//��ʼ�������Ƕ�
    G->YawMotor.Encoder->Init_Radian 	= G->YawMotor.Encoder->Radian;    			//��ʼ����ֵ��ֵ
    G->YawMotor.Encoder->Lock_Radian 	= G->YawMotor.Encoder->Radian;	    		//��ʼ�������Ƕ�
    G->YawMotor.MotorType = YAW_M;																						//��ʼ���������
    G->YawMotor.Radio = 19;																										//��ʼ�����̵�����ٱ�

    G->GimbalData = G->Get_Gimbal_Data();																			//��ȡ��̨�������ָ��

}



/*************************************************************************************************
*����:	Get_Gimbal_Data
*����:	��ȡ��̨�������
*�β�: 	G_t *G
*����:	��
*˵��:	��
*************************************************************************************************/
void Get_Gimbal_Data(G_t *G)
{
    G->PitchMotor.Speed 		= G->GimbalData[1];
    G->YawMotor.Speed 			= G->GimbalData[3];
    G->AmmunitiMotor.Speed 	= G->GimbalData[5];
}



/*************************************************************************************************
*����:	PY_Encoder_DRIVE
*����:	������̨PY��������ģʽ
*�β�: 	G_t *G,float P,float Y,float Y_SPEED
*����:	��
*˵��:	δ��������ֵ��ת��������
*************************************************************************************************/
void PY_Encoder_DRIVE(G_t *G, float P, float Y, float Y_SPEED)
{

    G->YawMotor.ExpRadian 	= Y;
    G->YawMotor.Speed 			= Y_SPEED;
    G->PitchMotor.ExpRadian = P;


    G->PitchMotor.PPID.Out = PID_DEAL_OVERSHOOT(&G->PitchMotor.PPID, G->PitchMotor.ExpRadian, G->PitchMotor.Encoder->Radian);
    G->PitchMotor.SPID.Out = PID_DEAL(&G->PitchMotor.SPID, G->PitchMotor.PPID.Out, G->PitchMotor.Speed);

    G->YawMotor.PPID.Out = PID_DEAL_OVERSHOOT(&G->YawMotor.PPID, G->YawMotor.ExpRadian, G->YawMotor.Encoder->Radian);
    G->YawMotor.SPID.Out = PID_DEAL(&G->YawMotor.SPID, G->YawMotor.PPID.Out, G->YawMotor.Speed);

    G->Can_Send_Gimbal(G->PitchMotor.SPID.Out, G->YawMotor.SPID.Out, G->AmmunitiMotor.SPID.Out, 0);
}



/*************************************************************************************************
*����:	Gimbal_Poweroff
*����:	��̨�ϵ�
*�β�: 	G_t *G
*����:	��
*˵��:	��
*************************************************************************************************/
void Gimbal_Poweroff(G_t *G)
{
    G->PitchMotor.SPID.Out = G->YawMotor.SPID.Out = G->AmmunitiMotor.SPID.Out = 0;
    G->Can_Send_Gimbal(0, 0, 0, 0);
}



/*************************************************************************************************
*����:	Ammuniti_Motor_Init
*����:	���������ʼ��
*�β�: 	G_t *G
*����:	��
*˵��:	��
*************************************************************************************************/
void Ammuniti_Motor_Init(G_t *G)
{
    MotorValZero(&G->AmmunitiMotor);
    G->AmmunitiMotor.ID = 8;																//ID��ֵ

    PID_INIT(&G->AmmunitiMotor.PPID, AMMUNITI_P_P, AMMUNITI_P_I, AMMUNITI_P_D, 0.0f, 0.0f);																				//�⻷��ʼ��
    PID_INIT(&G->AmmunitiMotor.SPID, AMMUNITI_S_P, AMMUNITI_S_I, AMMUNITI_S_D, 0.0f, 0.0f);    																		//�ڻ���ʼ��

    G->AmmunitiMotor.ExpRadian						= G->AmmunitiMotor.Encoder->Radian;					//��ʼ�������Ƕ�
    G->AmmunitiMotor.Encoder->Init_Radian = G->AmmunitiMotor.Encoder->Radian;    			//��ʼ����ֵ��ֵ
    G->AmmunitiMotor.Encoder->Lock_Radian = G->AmmunitiMotor.Encoder->Radian;	    		//��ʼ�������Ƕ�

    G->AmmunitiMotor.MotorType = AMMUNITI_M;																					//��ʼ���������
    G->AmmunitiMotor.Radio = 36;																											//��ʼ�����̵�����ٱ�
}



/*************************************************************************************************
*����:	Ammuniti_Ctrl
*����:	�������
*�β�: 	G_t *G,int8_t choice
*����:	��
*˵��:	��
*************************************************************************************************/
void Ammuniti_Ctrl(G_t *G, int8_t SpeedGain)
{
    if((SpeedGain > 0 && SpeedGain < 11) || (SpeedGain < 0 && SpeedGain > -11))
    {
        G->AmmunitiMotor.ExpSpeed = SpeedGain * 500;

    }
    else
    {
        G->AmmunitiMotor.ExpSpeed = 0;
    }

    G->AmmunitiMotor.SPID.Out = PID_DEAL(&G->AmmunitiMotor.SPID, G->AmmunitiMotor.ExpSpeed, G->AmmunitiMotor.Speed);

    G->Can_Send_Gimbal(G->PitchMotor.SPID.Out, G->YawMotor.SPID.Out, G->AmmunitiMotor.SPID.Out, 0);
}
