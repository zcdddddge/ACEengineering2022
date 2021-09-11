#include "Chassis_Fsm.h"
#include "Chassis_Task.h"
#include "BoardCommuni.h"

/*���̿����ܽṹ��*/
extern Chassis_t Chassis;
/*��������궨��*/
#define Electromagnet_On GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define Electromagnet_Off GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define Rifd_ForWard Chassis.Rescue(&Chassis.C, 2);
#define Rifd_BackWard Chassis.Rescue(&Chassis.C, 1);
#define Barrier_Up Chassis.Barrier(&Chassis.C, 1);
#define Barrier_Down Chassis.Barrier(&Chassis.C, -1);

FSM_t Chassis_FSM;                                     /*����״̬��*/
State_t OFFLINE;                                       /*�ϵ�ģʽ*/
State_t INDEPEN;                                       /*����ģʽ*/
State_t WIGGLE;                                        /*Ť��ģʽ*/
State_t ROTATION;                                      /*������ģʽ*/
State_t KEYBOARD;                                      /*����ģʽ*/
State_t CTRL_GRASP;                                    /*���Ƽ�ȡģʽ*/
State_t RESCUE;                                        /*��Ԯģʽ*/
State_t Chassis_State_Table[State_Line][State_Column]; /*״̬������*/

/***************************OFFLINE******************************/
static void Offline_State(void);   /*����״̬����*/
static void PowerOff_bhv(void);    /*�ϵ���Ϊ����*/
static void Offline_Prepare(void); /*OFFLINE״̬׼������*/
/***************************INDEPEN******************************/
static void Indepen_State(void);      /*����״̬����*/
static void Chassis_Normal_bhv(void); /*���������˶���Ϊ����*/
static void Indepen_Prepare(void);    /*INDEPEN״̬׼������*/
/***************************WIGGLE******************************/
static void Wiggle_State(void);      /*Ť��״̬����*/
static void Wiggle_Normal_bhv(void); /*����Ť���˶���Ϊ����*/
static void Wiggle_Prepare(void);    /*Wiggle״̬׼������*/
/***************************ROTATION******************************/
static void Rotation_State(void);      /*������״̬����*/
static void Rotation_Normal_bhv(void); /*�������˶���Ϊ����*/
static void Rotation_Prepare(void);    /*Rotation״̬׼������*/
/***************************KEYBOARD*****************************/
static void KeyBoard_State(void);       /*����״̬����*/
static void Upper_Device_bhv(void); /*��װ״̬*/
static void KeyBoard_Prepare(void);     /*KEYBOARD״̬׼������*/
/***************************CTRL_GRASP*****************************/
static void CTRL_GRASP_State(void);   /*���Ƽ�ȡ״̬����*/
static void GRASP_RC_bhv(void);       /*ң�ؿ��Ƽ�ȡ��Ϊ����*/
static void CTRL_GRASP_Prepare(void); /*CTRL_GRASP״̬׼������*/
/****************************RESCUE-2020-11-18-author:wingchi-leung*********************/
static void Rescue_State(void);   /*��Ԯ״̬����*/
static void Rescue_Prepare(void); /*��Ԯ״̬׼��*/
static void Rescue_bhv(void);     /*��Ԯ��Ϊ����*/

/*���ص���״̬������ָ��*/
FSM_t *Return_Chassis_FSM(void)
{
    return &Chassis_FSM;
}

/*����״̬����ʼ��*/
void Chassis_FSM_Init(void)
{
    Chassis_FSM.State_Table = Chassis_State_Table;
    Chassis_FSM.Last_State = NULL;
    Chassis_FSM.Current_State = NULL;
    Chassis_FSM.State_Change = StateChange; //״̬��״̬�������

    /*OFFLINE״̬��ʼ��*/
    OFFLINE.Behavior_Process = NULL;
    OFFLINE.State_Process = Offline_State;
    OFFLINE.State_Prepare = Offline_Prepare;
    /*INDEPEN״̬��ʼ��*/
    INDEPEN.Behavior_Process = NULL;
    INDEPEN.State_Process = Indepen_State;
    INDEPEN.State_Prepare = Indepen_Prepare;
    /*WIGGLE״̬��ʼ��*/
    WIGGLE.Behavior_Process = NULL;
    WIGGLE.State_Process = Wiggle_State;
    WIGGLE.State_Prepare = Wiggle_Prepare;
    /*ROTATION״̬��ʼ��*/
    ROTATION.Behavior_Process = NULL;
    ROTATION.State_Process = Rotation_State;
    ROTATION.State_Prepare = Rotation_Prepare;
    /*KEYBOARD״̬��ʼ��*/
    KEYBOARD.Behavior_Process = NULL;
    KEYBOARD.State_Process = KeyBoard_State;
    KEYBOARD.State_Prepare = KeyBoard_Prepare;
    /*CTRL_GRASP״̬��ʼ��*/
    CTRL_GRASP.Behavior_Process = NULL;
    CTRL_GRASP.State_Process = CTRL_GRASP_State;
    CTRL_GRASP.State_Prepare = CTRL_GRASP_Prepare;
    /*RESCUE״̬��ʼ��*/
    RESCUE.Behavior_Process = NULL;
    RESCUE.State_Process = Rescue_State;
    RESCUE.State_Prepare = Rescue_Prepare;

    /*����״̬����ʼ��*/
    Chassis_State_Table[0][0] = INDEPEN;    //s1=1 ,s2=1 ���̶���INDEPEN
    Chassis_State_Table[0][2] = INDEPEN;     //s1=1  s2=3 Ť��WIGGLE
    Chassis_State_Table[0][1] = INDEPEN;    //s1=1  s2=2 ����
    Chassis_State_Table[1][0] = OFFLINE;     //s1=2  s2=1 RESCUE��ʱ--�ϰ������
    Chassis_State_Table[1][1] = OFFLINE; //s1=2  s2=2 RESCUE ��Ԯ������
    Chassis_State_Table[1][2] = CTRL_GRASP; //s1=2  s2=3 ��ȡ
    Chassis_State_Table[2][0] = KEYBOARD;   //s1=3 s2=1  ����
    Chassis_State_Table[2][1] = KEYBOARD; //s1=3 s2=2  ��ȡ
    Chassis_State_Table[2][2] = KEYBOARD; //s1=3 s2=3  ��ȡ
}

/********************************RESCUE**********************************************/
static void Rescue_State(void)
{
    Chassis_FSM.Current_State->Behavior_Process = Rescue_bhv;
}

static void Rescue_Prepare(void)
{
}
static void Rescue_bhv(void)
{
    //    float  s = ((float)(Chassis.RC->RC_ctrl->ch0) * 10.0f) ;
    //	if(Chassis.RC->RC_ctrl->s2 ==0 )
    //    Chassis.Barrier(&Chassis.C, s);
    //	else if  (Chassis.RC->RC_ctrl->s2 == 1)
    //		Chassis.Barrier(&Chassis.C, -1 );
    //	Chassis.Indepen(&Chassis.C,0,0,0,0);
}

/***************************************OFFLINE**************************************/
/*OFFLINE״ִ̬�к���*/
static void Offline_State(void)
{
    Chassis_FSM.Current_State->Behavior_Process = PowerOff_bhv;
}

/*OFFLINE״̬׼������*/
static void Offline_Prepare(void)
{
    Chassis.C.WheelMotor[0].SPID.Out = Chassis.C.WheelMotor[1].SPID.Out = Chassis.C.WheelMotor[2].SPID.Out = Chassis.C.WheelMotor[3].SPID.Out = 0;
    Send_RC_To_Board(); //����ң������
}

/*�ϵ���Ϊ����*/
static void PowerOff_bhv(void)
{
    Chassis.Poweroff(&Chassis.C);
}

/***************************************INDEPEN**************************************/
/*����״̬����*/
static void Indepen_State(void)
{
    Chassis_FSM.Current_State->Behavior_Process = Chassis_Normal_bhv;
}

/*���������˶���Ϊ����*/
static void Chassis_Normal_bhv(void)
{
    Electromagnet_On;
    Chassis.Indepen(&Chassis.C, Chassis.RC->RC_X.Output, Chassis.RC->RC_Y.Output, Chassis.RC->RC_Z.Output, 0);
}

/*INDEPEN״̬׼������*/
static void Indepen_Prepare(void)
{
    Chassis.C.gyro->Yaw_Lock = Chassis.C.gyro->Yaw;
}

/***************************************WIGGLE**************************************/
/*Ť��״̬����*/
static void Wiggle_State(void)
{
    Chassis_FSM.Current_State->Behavior_Process = Wiggle_Normal_bhv;
}

/*����Ť���˶���Ϊ����*/
static void Wiggle_Normal_bhv(void)
{
    Chassis.Wiggle_Run(&Chassis.C, Chassis.RC->RC_X.Output, Chassis.RC->RC_Y.Output, 0.0f);
}

/*Wiggle״̬׼������*/
static void Wiggle_Prepare(void)
{
    Chassis.C.gyro->Yaw_Lock = Chassis.C.gyro->Yaw;
}

/***************************************ROTATION**************************************/
/*����״̬����*/
static void Rotation_State(void)
{
    Chassis_FSM.Current_State->Behavior_Process = Rotation_Normal_bhv;
}

/*���������˶���Ϊ����*/
static void Rotation_Normal_bhv(void)
{
    Chassis.Rotation(&Chassis.C, Chassis.RC->RC_X.Output, Chassis.RC->RC_Y.Output, 0.0f);
}

/*Rotation״̬׼������*/
static void Rotation_Prepare(void)
{
    Chassis.C.gyro->Yaw_Lock = Chassis.C.gyro->Yaw;
}

/***************************************KeyBoard**************************************/
/*����״̬����*/
static void KeyBoard_State(void)
{
    /***************************************����ѡ��**********************************/
    if (Chassis.RC->state.Independent == 0 || Chassis.RC->state.Rotation == 0 || Chassis.RC->state.Wiggle == 0)
    {
        Chassis.Poweroff(&Chassis.C);
    }

    if (Chassis.RC->state.Independent == 1)
    {
        Chassis.Indepen(&Chassis.C, Chassis.RC->KM_X.Output, Chassis.RC->KM_Y.Output, Chassis.RC->KM_Z.Output ,0);
    }

    if (Chassis.RC->state.Rotation == 1)
    {
        Chassis.Rotation(&Chassis.C, Chassis.RC->KM_X.Output, Chassis.RC->KM_Y.Output, Chassis.RC->KM_Z.Output);
    }

    if (Chassis.RC->state.Wiggle == 1)
    {
        Chassis.Wiggle_Run(&Chassis.C, Chassis.RC->KM_X.Output, Chassis.RC->KM_Y.Output, Chassis.RC->KM_Z.Output);
				Chassis.C.gyro->Yaw_Lock = Chassis.C.gyro->Yaw;
    }


    /***************************************����ѡ��**********************************/
    if (Chassis.RC->state.RFID)
    {
        Rifd_ForWard;
    }
    else
    {
        Rifd_BackWard;
    }

    if (Chassis.RC->state.Barrier)
    {
        Barrier_Up;
    }
    else
    {
        Barrier_Down;
    }

    /***************************************��װѡ��**********************************/
    Chassis_FSM.Current_State->Behavior_Process = Upper_Device_bhv;
}

/***************************************��װ״̬**********************************/
static void Upper_Device_bhv(void)
{
    Send_Ctrl_To_Board(Chassis.RC->state.Grasp_Up, Chassis.RC->state.Translation, Chassis.RC->state.Telescoping, Chassis.RC->state.Clap, Chassis.RC->state.Flip);
    //Send_RC_To_Board(); //����ң������
}

/* ======================================================�ָ���======================================================================================= */



#if 0
/*���̿��Ƶ�����Ϊ����*/
static void KeyBoard_Chassis_bhv(void)
{

    Chassis.Indepen(&Chassis.C, Chassis.RC->KM_X.Output, Chassis.RC->KM_Y.Output, Chassis.RC->KM_Z.Output, 0);

    //�����
    if (Chassis.RC->state.Electromagnet)
    {
        Electromagnet_On;
    }
    else
    {
        Electromagnet_Off;
    }

    //��Ԯ��
    if (Chassis.RC->state.RFID)
    {
        Rifd_ForWard;
    }
    else
    {
        Rifd_BackWard;
    }
    //�ϰ���
    if (Chassis.RC->state.Barrier)
    {
        Barrier_Up;
    }
    else
    {
        Barrier_Down;
    }
}

/*���̿��Ƽ�ȡ��Ϊ����*/
static void KeyBoard_Grasp_bhv(void)
{
    /*����������־λ����ȡ*/
    Send_Ctrl_To_Board(Chassis.RC->state.Auto_Clamp, Chassis.RC->state.Magazine, Chassis.RC->state.Up_motor, Chassis.RC->state.Flex_motor, Chassis.RC->state.Flip_motor, Chassis.RC->state.Clamp_motor);
    /*���̿��Ƶ���*/
    Chassis.Indepen(&Chassis.C, Chassis.RC->KM_X.Output, Chassis.RC->KM_Y.Output, Chassis.RC->KM_Z.Output, 0);
}

#endif
/*KEYBOARD״̬׼������*/
static void KeyBoard_Prepare(void)
{
    Chassis.C.gyro->Yaw_Lock = Chassis.C.gyro->Yaw;
}

/***************************************CTRL_GRASP**************************************/
/*���Ƽ�ȡ״̬����*/
static void CTRL_GRASP_State(void)
{
    Chassis_FSM.Current_State->Behavior_Process = GRASP_RC_bhv;
}

/*ң�ؿ��Ƽ�ȡ��Ϊ����*/
static void GRASP_RC_bhv(void)
{
    Send_RC_To_Board(); //����ң������
    Chassis.Indepen(&Chassis.C, 0, 0, 0, 0);
}

/*CTRL_GRASP״̬׼������*/
static void CTRL_GRASP_Prepare(void)
{
    Chassis.Indepen(&Chassis.C, 0, 0, 0, 0); //�����ٶ�Ϊ0
}

#if 0
static void Gimbal_State(void)
{
    Chassis_FSM.Current_State->Behavior_Process = Gimbal_bhv;
}

static void Gimbal_Prepare(void)
{
}
static void Gimbal_bhv(void)
{
    PY_Encoder_DRIVE(&Chassis.G, Chassis.RC->RC_X.Output, Chassis.RC->RC_Y.Output, Chassis.G.YawMotor.Encoder->Speed[1]);
}
#endif
