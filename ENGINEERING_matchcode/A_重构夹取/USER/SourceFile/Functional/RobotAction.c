/*
 * @Date: 2021-02-24 11:38:12
 * @LastEditTime: 2021-04-12 20:11:20
 * @LastEditors: Please set LastEditors
 * @Description: �����˹����Զ���ȡ���ƺ���,�ص���״̬����
 */

#include "RobotAction.h"


extern Sensor_t Sensor;

#if 0

/**
 * @description: �Զ���ȡ��������,�ص�����״̬
 * @param {Gr_t} *Gr
 * @param {u8} box ������
 * @return {*}
 * @note 201-̧��	202-ƽ��	203-����	204-����	205-��ת	206-��ת	207-����
 * 	 0 ̧��   2����   3 �н�  4 ��ת  7��ת
 * @note ���¹���δ����  ̧��Ҫ���¹����޲��ģ��ĺ���
 */
void Auto_Ctrl(Gr_t *Gr, u8 box)
{
    static u8 box_lock = 1;
    static u8 boxs = 0;

    /*���������������¸�ֵ����(��ֹ�����������)*/
    if(box_lock == 1)
    {
        boxs = box;
        box_lock = 2;

        if(boxs != 0)
            Gr->GraspMotor[0].state = DisFinish;//Ԥ��̧��
    }

    /*������ĿΪ1*/
    if(boxs == 1)
    {
        if(Gr->state[0] != 4)
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 10.0f, 1);		//��ȡǰ̧��׼��

            if(UPLIFT.state == Finish)
            {
                Auto_One_Box(Gr);
            }
        }
        else if(Gr->state[0] == 4)
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 1.0f, 2);		//��ȡ��Ż�

            if(Gr->GraspMotor[0].state == Finish)
            {
                Gr->GraspMotor[3].state = DisFinish;
                Gr->GraspMotor[4].state = DisFinish;
                Gr->GraspMotor[5].state = DisFinish;
                box_lock = 1;
                boxs = 0;
                Gr->state[0] = 0;
            }
        }
    }



    else if(boxs == 2)/*������ĿΪ2*/
    {
        if(Gr->state[1] != 4)
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 13.5f, 1);		//��ȡǰ̧��׼��

            if(Gr->GraspMotor[0].state == Finish)
            {
                Auto_Two_Box(Gr);
            }
        }
        else if(Gr->state[1] == 4)
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 4.5f, 2);		//��ȡ��Ż�

            if(Gr->GraspMotor[0].state == Finish)
            {
                Gr->GraspMotor[1].ExpSpeed = 0;							//ƽ���ٶ���0
                Gr->GraspMotor[1].state = DisFinish;				//ƽ�Ʊ��Ϊδ���
                Gr->GraspMotor[3].ExpSpeed = 0;
                Gr->GraspMotor[3].state = DisFinish;
                Gr->GraspMotor[4].state = DisFinish;
                Gr->GraspMotor[5].state = DisFinish;
                box_lock = 1;                             // ���ӽ���
                boxs = 0;
                Gr->state[1] = 0;
                //��ʱ�����жϽ���
            }
        }
    }
    else if(boxs == 0)
    {
        box_lock = 1;
    }

    pid_Cala(Gr);
}



/**
 * @description: �Զ���ȡһ��
 * @param {*}
 * @return {*}
 */
static void Auto_One_Box(Gr_t *Gr)
{
    switch (Gr->state[0])
    {
        /*ǰ��*/
        case 0:
        {
            flip(&Gr->GraspMotor[4], &Gr->GraspMotor[5], 182.0f, 6.0f, 1);

            if(Gr->GraspMotor[4].state == Finish && Gr->GraspMotor[5].state == Finish)
            {
                Gr->GraspMotor[3].ExpSpeed = 0;
                Gr->GraspMotor[3].state = DisFinish;					//���ӱ�־Ϊδ���
                Gr->state[0] = 1;
            }

            break;
        }

        /*�н�*/
        case 1:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 1);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;					//��ת���Ϊδ���
                Gr->GraspMotor[5].state = DisFinish;
                Gr->state[0] = 2;
            }

            break;
        }

        /*��*/
        case 2:
        {
            flip(&Gr->GraspMotor[4], &Gr->GraspMotor[5], -160.0f, 20.0f, 2); // -170-->-160  ���

            if(Gr->GraspMotor[4].state == Finish && Gr->GraspMotor[5].state == Finish)
            {
                Gr->GraspMotor[3].ExpSpeed = 0;
                Gr->GraspMotor[3].state = DisFinish;					//���ӱ�־Ϊδ���
                Gr->state[0] = 3;
            }

            break;
        }

        /*�ɼ�*/
        case 3:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 2);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[3].ExpSpeed = 0;
                Gr->GraspMotor[3].state = DisFinish;				//���ӱ��Ϊδ���
                Gr->GraspMotor[0].state = DisFinish;				//̧�����Ϊδ���
                Gr->state[0] = 4;
            }

            break;
        }

        default:
        {
            break;
        }
    }
}






static void Auto_Two_Box(Gr_t *Gr)
{
    switch (Gr->state[1])
    {
        /*ƽ��*/
        case 0:
        {
            rail(&Gr->GraspMotor[1], Gr->sensor, 1);

            if(Gr->GraspMotor[1].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;					//��ת���Ϊδ���
                Gr->GraspMotor[5].state = DisFinish;
                Gr->state[0] = 0;															//��ȡһ��Ϊ���¿�ʼ
                Gr->state[1] = 1;
            }

            break;
        }

        /*��ȡһ��*/
        case 1:
        {
            Auto_One_Box(Gr);

            if(Gr->state[0] == 4)
            {
                Gr->GraspMotor[0].state = Finish;						//̧�����Ϊ���
                Gr->GraspMotor[1].state = DisFinish;				//ƽ�Ʊ��Ϊδ���
                Gr->state[1] = 2;
            }

            break;
        }

        /*ƽ��*/
        case 2:
        {
            rail(&Gr->GraspMotor[1], Gr->sensor, 1);

            if(Gr->GraspMotor[1].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;					//��ת���Ϊδ���
                Gr->GraspMotor[5].state = DisFinish;
                Gr->state[0] = 0;															//��ȡһ��Ϊ���¿�ʼ
                Gr->state[1] = 3;
            }

            break;
        }

        /*��ȡһ��*/
        case 3:
        {
            Auto_One_Box(Gr);

            if(Gr->state[0] == 4)
            {
                Gr->GraspMotor[0].state = DisFinish;						//̧�����Ϊ���
                Gr->state[0] = 0;
                Gr->state[1] = 4;
            }

            break;
        }

        default:
            break;

    }
}
#endif

static void Grasp_init(Gr_t *Gr);
static void	Grasp_First(Gr_t *Gr);
static void	Grasp_Second(Gr_t *Gr);
static void	Grasp_Third(Gr_t *Gr);

/*================================================================================================
------------------------------------------����Ϊ�µĴ���------------------------------------------
================================================================================================*/
void auto_grasp(Gr_t *Gr)
{
	if(Gr->state[0] < 3)
	{
		Grasp_init(Gr);
	}
	
	if(Gr->state[0] >= 3 && Gr->state[0] < 9)
	{
		Grasp_First(Gr);
	}
	
	if(Gr->state[0] >= 9 && Gr->state[0] < 16)
	{
		Grasp_Second(Gr);
	}
	
	if(Gr->state[0] >= 16 && Gr->state[0] < 20)
	{
		Grasp_Third(Gr);
	}
}



/**
 * @description:��ȡ����ʼ��
 * @param {Gr_t} *Gr
 * @param
 * @return {*}
 * @note 201-̧��	202-ƽ��	203-����	204-����	205-��ת	206-��ת	207-����
 * 	 0 ̧��   2����   3 �н�  4 ��ת  7��ת
 * @note ���˺���Ϊ����̧����ǰ�� ̧������ĸ߶Ȼ�Ҫ�ٲ�
 */
static void Grasp_init(Gr_t *Gr)
{
    switch (Gr->state[0])
    {
        /*̧��*/
        case 0:
        {
            Gr->GraspMotor[0].state = DisFinish;//Ԥ��̧��

            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 10.0f, 1);

            if(Gr->GraspMotor[0].state == Finish)
            {
                Gr->GraspMotor[1].state = DisFinish;//Ԥ��ƽ��
                Gr->GraspMotor[4].state = DisFinish;//Ԥ����ȡ
                Gr->state[0] = 1;
            }

            break;
        }

        /*ǰ�� ��ת*/
        case 1:
        {
            Translation(&Gr->GraspMotor[1], 1);//ƽ��
            if(Gr->GraspMotor[1].state == Finish)//
            {
                Gr->GraspMotor[2].state = DisFinish;//Ԥ����ȡ
                Gr->state[0] = 2;
            }

            break;
        }

        /*ǰ��*/
        case 2:
        {

                Telescoping(&Gr->GraspMotor[2], 1);

                if(Gr->GraspMotor[2].state == Finish)
                {
										flip2(&Gr->GraspMotor[4], 170, 10, 1);//��ת��90��
                    Gr->GraspMotor[4].state = DisFinish;
                    Gr->state[0] = 3;
                }
            }

            break;
        
				
        default:
            break;
    }

    pid_Cala(Gr);
}


static void Grasp_reset(Gr_t *Gr)
{
		Gr->GraspMotor[0].state = DisFinish;//Ԥ���½�
	
		uplift(&Gr->GraspMotor[0], Gr->vL53L0, 1.0f, 2);
	
		if(Gr->GraspMotor[0].state == Finish)
		{
				Gr->GraspMotor[1].state = DisFinish;//Ԥ��ƽ��
        Gr->GraspMotor[4].state = DisFinish;//Ԥ����ȡ
		}
		
		if(Gr->GraspMotor[1].state == Finish)
		{
				Translation(&Gr->GraspMotor[1], 2);//Ԥ��ƽ��
		}
		
		if(Gr->GraspMotor[4].state == Finish)
		{
				flip2(&Gr->GraspMotor[4], 0, 10, 2);//��ת����ʼ
		}
}


static void Grasp_First(Gr_t *Gr)
{
    switch (Gr->state[0])
    {
        /*ǰ��*/
        case 3:
        {
					if(Sensor.Smooth_L && Sensor.Smooth_R)
          {
            flip2(&Gr->GraspMotor[4], 170, 10, 1);

            if(Gr->GraspMotor[4].state == Finish)
            {
                Gr->GraspMotor[3].state = DisFinish;
                Gr->state[0] = 4;
            }
					}
            break;
        }

        /*�н�*/
        case 4:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 1);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;
                Gr->state[0] = 5;
            }

            break;
        }

        /*��*/
        case 5:
        {
            flip2(&Gr->GraspMotor[4], 0, 10, 2);

            if(Gr->GraspMotor[4].state == Finish)
            {
                Gr->GraspMotor[2].state = DisFinish;
                Gr->state[0] = 6;
            }

            break;
        }

        /*����*/
        case 6:
        {
            Telescoping(&Gr->GraspMotor[2], 2);

            if(Gr->GraspMotor[2].state == Finish)
            {
                Gr->GraspMotor[3].state = DisFinish;
                Gr->state[0] = 7;
            }

            break;
        }

        /*�ɼ�*/
        case 7:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 2);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[1].state = DisFinish;
                Gr->GraspMotor[4].state = DisFinish;
                Gr->state[0] = 8;
            }

            break;
        }

        /*���� ��ת*/
        case 8:
        {
            Translation(&Gr->GraspMotor[1], 2);//Ԥ��ƽ��
            flip2(&Gr->GraspMotor[4], 65, 10, 1);//Ԥ����ת��90��

            if(Gr->GraspMotor[1].state == Finish || Gr->GraspMotor[4].state == Finish)
            {
                Gr->state[0] = 9;//��λ���ȴ���һ����ȡָ��
            }

            break;
        }

        default:
            break;
    }

    pid_Cala(Gr);
}

static void Grasp_Second(Gr_t *Gr)
{
    switch (Gr->state[0])
    {
        /*��ʼ������˲��ܽ��м�ȡ*/
        /*ǰ��*/
        case 9:
        {
            if(Sensor.Smooth_L || Sensor.Smooth_R)
            {
                Telescoping(&Gr->GraspMotor[2], 1);

                if(Gr->GraspMotor[2].state == Finish)
                {
                    Gr->GraspMotor[4].state = DisFinish;
                    Gr->state[0] = 10;
                }
            }

            break;
        }

        /*ǰ��*/
        case 10:
        {
            flip2(&Gr->GraspMotor[4], 170, 10, 1);

            if(Gr->GraspMotor[4].state == Finish)
            {
                Gr->GraspMotor[3].state = DisFinish;
                Gr->state[0] = 11;
            }

            break;
        }

        /*�н�*/
        case 11:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 1);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[2].state = DisFinish;
                Gr->state[0] = 12;
            }

            break;
        }

        /*����*/
        case 12:
        {
            Telescoping(&Gr->GraspMotor[2], 2);

            if(Gr->GraspMotor[2].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;
                Gr->state[0] = 13;
            }

            break;
        }

        /*��*/
        case 13:
        {
            flip2(&Gr->GraspMotor[4], 0, 10, 2);

            if(Gr->GraspMotor[4].state == Finish)
            {
                Gr->GraspMotor[3].state = DisFinish;
                Gr->state[0] = 14;
            }

            break;
        }

        /*�ɼ�*/
        case 14:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 2);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;
                Gr->state[0] = 15;
            }

            break;
        }

        /*��ת*/
        case 15:
        {
            flip2(&Gr->GraspMotor[4], 65, 10, 1);//Ԥ����ת��90��

            if(Gr->GraspMotor[4].state == Finish)
            {
                Gr->state[0] = 16;//��λ���ȴ���һ����ȡָ��
            }

            break;
        }

        default:
            break;
    }

    pid_Cala(Gr);
}


static void Grasp_Third(Gr_t *Gr)
{
    switch (Gr->state[0])
    {
        /*��ʼ������˲��ܽ��м�ȡ*/
        /*ǰ��*/
        case 16:
        {
            if(Sensor.Smooth_L || Sensor.Smooth_R)
            {
                Telescoping(&Gr->GraspMotor[2], 1);

                if(Gr->GraspMotor[2].state == Finish)
                {
                    Gr->GraspMotor[4].state = DisFinish;
                    Gr->state[0] = 17;
                }
            }

            break;
        }

        /*ǰ��*/
        case 17:
        {
            flip2(&Gr->GraspMotor[4], 170, 10, 1);

            if(Gr->GraspMotor[4].state == Finish)
            {
                Gr->GraspMotor[3].state = DisFinish;
                Gr->state[0] = 18;
            }

            break;
        }

        /*�н�*/
        case 18:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 1);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[2].state = DisFinish;
                Gr->state[0] = 19;
            }

            break;
        }

        /*����*/
        case 19:
        {
            Telescoping(&Gr->GraspMotor[2], 2);

            if(Gr->GraspMotor[2].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;
                Gr->state[0] = 2;
            }

            break;
        }

        default:
            break;
    }

    pid_Cala(Gr);
}

/**
 * @description: ��ȡʧ�ܸ�λ:���ǵ�һ�㶼��ǰ��ʱ������-ִֻ�к�,������̧��,û�иı�����״̬
 * @param {Gr_t} *Gr
 * @return {*}
 * @note
 * 		 4.12 �����˲���Ч����Ҫ���²�
 */
void Reset(Gr_t *Gr)
{

    flip2(&Gr->GraspMotor[4], 20.0f, 10, 2);

    if(Gr->GraspMotor[0].state == Finish)
    {
        PID_DEAL(&Gr->GraspMotor[0].PPID, Gr->GraspMotor[0].Encoder->Lock_Radian, Gr->GraspMotor[0].Encoder->Total_Radian);										//�⻷
        PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].PPID.Out, Gr->GraspMotor[0].Encoder->Speed[0]);
    }

    PID_DEAL(&Gr->GraspMotor[4].PPID, Gr->GraspMotor[4].ExpRadian, Gr->GraspMotor[4].Encoder->Total_Radian);
    PID_DEAL(&Gr->GraspMotor[5].PPID, Gr->GraspMotor[5].ExpRadian, Gr->GraspMotor[5].Encoder->Total_Radian);

    Gr->Can_Send_Grasp_1(MotorOutput_201_204);
    Gr->Can_Send_Grasp_2(MotorOutput_205_208);
}



