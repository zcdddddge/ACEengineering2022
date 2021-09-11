/*
 * @file: GraspMotor.c
 * @description: �����˼�ȡ�����ʼ��,ң�ؿ���,��λ,��������
 * @date:
 * @history:
 */


#include "GraspMotor.h"

/**
 * @description: ��ȡ�����ֳ�ʼ��
 * @param {Gr_t} *Gr
 * @return {*}
 * @note 201-̧��	202-��һ	203-����	204-����	205-��ת	206-��λ��ת	207-����  208-��ת
 */
void Grasp_Motor_Init(Gr_t *Gr)
{
    u8 i = 0;
    float Spid[8][3] =
    {
        {GRASP_UPLIFT_S_P, GRASP_UPLIFT_S_I, GRASP_UPLIFT_S_D},
        {GRASP_CENTER_S_P, GRASP_CENTER_S_I, GRASP_CENTER_S_D},
        {GRASP_TRANS_S_P, GRASP_TRANS_S_I, GRASP_TRANS_S_D},
        {GRASP_CLIP_S_P, GRASP_CLIP_S_I, GRASP_CLIP_S_D},
        {GRASP_FLIP_S_P, GRASP_FLIP_S_I, GRASP_FLIP_S_D},
        {GRASP_NOROATE_S_P, GRASP_NOROATE_S_I, GRASP_NOROATE_S_D},
        {GRASP_SUPPLY_S_P, GRASP_SUPPLY_S_I, GRASP_SUPPLY_S_D},
        {GRASP_ROTATE_S_P, GRASP_ROTATE_S_I, GRASP_ROTATE_S_D}
    };
    float Ppid[8][3] =
    {
        {GRASP_UPLIFT_P_P, GRASP_UPLIFT_P_I, GRASP_UPLIFT_P_D},
        {GRASP_CENTER_P_P, GRASP_CENTER_S_I, GRASP_CENTER_P_D},
        {GRASP_TRANS_P_P, GRASP_TRANS_P_I, GRASP_TRANS_P_D},
        {GRASP_CLIP_P_P, GRASP_CLIP_P_I, GRASP_CLIP_P_D},
        {GRASP_FLIP_P_P, GRASP_FLIP_P_I, GRASP_FLIP_P_D},
        {GRASP_NOROATE_P_P, GRASP_NOROATE_P_I, GRASP_NOROATE_P_D},
        {GRASP_SUPPLY_P_P, GRASP_SUPPLY_P_I, GRASP_SUPPLY_P_D},
        {GRASP_ROTATE_P_P, GRASP_ROTATE_P_I, GRASP_ROTATE_P_D}
    };

    /*����ӳ��*/
    Gr->Can_Send_Grasp_1			= CAN1_201_To_204_SEND;
    Gr->Can_Send_Grasp_2	 		= CAN1_205_To_208_SEND;
    Gr->Get_Encoder						=	Return_Can1_201_208_Encoder;
		Gr->Get_Encoder2						=	Return_Can2_201_208_Encoder;
    Gr->Get_Sensor_t					= Return_Sensor_t;
    Gr->Get_VL53L0_t					= Return_VL53L0_t;

    for(i = 0; i < 8; i ++)
    {
        /*���㴦��*/
        MotorValZero(&Gr->GraspMotor[i]);
        /*ID��ֵ*/
        Gr->GraspMotor[i].ID = i + 1;
        /*���̸�ֵ*/
        Gr->GraspMotor[i].Encoder = Gr->Get_Encoder(i + 1);
        PID_INIT(&Gr->GraspMotor[i].PPID, Ppid[i][0], Ppid[i][1], Ppid[i][2], 0, 8500);							//λ�û���ʼ��
        PID_INIT(&Gr->GraspMotor[i].SPID, Spid[i][0], Spid[i][1], Spid[i][2], 0, 16000);							//�ٶȻ���ʼ��
        Gr->GraspMotor[i].ExpRadian							= Gr->GraspMotor[i].Encoder->Radian;						//��ʼ�������Ƕ�
        Gr->GraspMotor[i].Encoder->Init_Radian 	= Gr->GraspMotor[i].Encoder->Radian;    				//��ʼ����ֵ��ֵ
        Gr->GraspMotor[i].Encoder->Lock_Radian 	= Gr->GraspMotor[i].Encoder->Radian;	    			//��ʼ�������Ƕ�
        Gr->GraspMotor[i].Encoder->Total_Radian = Gr->GraspMotor[i].Encoder->Radian;	    			//��ʼ���ܽǶ�
        Gr->GraspMotor[i].Radio = 19;																														//��ʼ����ȡ������ٱ�
        Gr->GraspMotor[i].ResetFlag = DisFinish ;
        Gr->GraspMotor[i].debug = 0;
    }

    PID_INIT(&Gr->GraspMotor[2].PPID, Ppid[2][0], Ppid[2][1], Ppid[2][2], 0, 14700);							//λ�û���ʼ��
		PID_INIT(&Gr->GraspMotor[3].SPID, Ppid[3][0], Ppid[3][1], Ppid[3][2], 0, 16000);
    PID_INIT(&Gr->GraspMotor[6].PPID, Ppid[6][0], Ppid[6][1], Ppid[6][2], 0, 14700);							//λ�û���ʼ��

    Gr->GraspMotor[0].Radio = 27;																														//��ʼ��̧��������ٱ�
    Gr->GraspMotor[2].Radio = 36;																														//��ʼ������������ٱ�
    Gr->GraspMotor[3].Radio = 19;																														//��ʼ����ȡ������ٱ�
    Gr->GraspMotor[6].Radio = 36;																														//��ʼ������������ٱ�
    Gr->GraspMotor[7].Radio = 36;


    /*״ֵ̬��ʼ��*/
    Gr->state[0] = Gr->state[1] = Gr->state[2] = Gr->state[3] = DisFinish;
		Gr->conversion_state = DisFinish;
		
    /*��λֵ��ʼ��*/
    Gr->reset[0] = Gr->reset[1] = Gr->reset[2] = Gr->reset[3] \
                                  = Gr->reset[4] = Gr->reset[5] = Gr->reset[6] = DisFinish;

    /*��������ʼ��*/
    Gr->sensor	= Gr->Get_Sensor_t();
    Gr->sensor->Smooth_L = Gr->sensor->Smooth_R = Gr->sensor->Stretch_F = 2;

    /*VL53L0����ʼ��*/
    Gr->vL53L0 = Gr->Get_VL53L0_t();
		
		
}



/**
 * @description: ң�ؿ���
 * @param {Gr_t} *Gr
 * @param {RC_ctrl_t} *r
 * @return {*}
 * @note 201-̧��	202-��λ��һ	203-����	204-����	205-��ת	206-��λ��һ	207-����
 *	 0 ̧��   2����   3 �н�  4 ��ת  7��ת
 */
void RC_Ctrl(Gr_t *Gr, RC_ctrl_t *rc)
{
    static int16_t clock = 0;
    static int8_t dire = 0;
    static int8_t lock = 1;


    Gr->GraspMotor[0].ExpSpeed   = -((float)(rc->ch1) * 10.0f); // ̧��
    Gr->GraspMotor[1].ExpSpeed   = -((float)(rc->ch0) * 15.0f); // ƽ��
    Gr->GraspMotor[2].ExpSpeed   = ((float)(rc->ch3) * 10.0f); // ����
    Gr->GraspMotor[3].ExpSpeed   = -((float)(rc->ch2) * 10.0f);  // �н�
    Gr->GraspMotor[4].ExpRadian += ((float)(rc->sw) / 1600.0f); // ��ת
    //Gr->GraspMotor[6].ExpSpeed   = ((float)(rc->ch1) * 10.0f);// ����
    //Gr->GraspMotor[7].ExpSpeed   = ((float)(rc->ch3) * 10.0f);  // ��Ԯצ





    /****************************̧��***************************/
    if(Gr->GraspMotor[0].ExpSpeed == 0)
    {
        /*�⻷*/
        PID_DEAL(&Gr->GraspMotor[0].PPID, Gr->GraspMotor[0].Encoder->Init_Radian, Gr->GraspMotor[0].Encoder->Total_Radian);
        /*�ڻ�*/
        PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].PPID.Out, Gr->GraspMotor[0].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].ExpSpeed, Gr->GraspMotor[0].Encoder->Speed[1]);
        Gr->GraspMotor[0].Encoder->Init_Radian = Gr->GraspMotor[0].Encoder->Total_Radian;
    }

    /***************************��λ**************************/
    if(Gr->GraspMotor[1].ExpSpeed == 0)
    {
        /*�⻷*/
        PID_DEAL(&Gr->GraspMotor[1].PPID, Gr->GraspMotor[1].Encoder->Init_Radian, Gr->GraspMotor[1].Encoder->Total_Radian);
        /*�ڻ�*/
        PID_DEAL(&Gr->GraspMotor[1].SPID, Gr->GraspMotor[1].PPID.Out, Gr->GraspMotor[1].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[1].SPID, Gr->GraspMotor[1].ExpSpeed, Gr->GraspMotor[1].Encoder->Speed[1]);
        Gr->GraspMotor[1].Encoder->Init_Radian = Gr->GraspMotor[1].Encoder->Total_Radian;
    }

    /***************************����**************************/
    if(Gr->GraspMotor[2].ExpSpeed == 0)
    {
        /*�⻷*/
        PID_DEAL(&Gr->GraspMotor[2].PPID, Gr->GraspMotor[2].Encoder->Init_Radian, Gr->GraspMotor[2].Encoder->Total_Radian);
        /*�ڻ�*/
        PID_DEAL(&Gr->GraspMotor[2].SPID, Gr->GraspMotor[2].PPID.Out, Gr->GraspMotor[2].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[2].SPID, Gr->GraspMotor[2].ExpSpeed, Gr->GraspMotor[2].Encoder->Speed[1]);
        Gr->GraspMotor[2].Encoder->Init_Radian = Gr->GraspMotor[2].Encoder->Total_Radian;
    }

    /************************����*******************************/
    if(Gr->GraspMotor[6].ExpSpeed == 0)
    {
        /*�⻷*/
        PID_DEAL(&Gr->GraspMotor[6].PPID, Gr->GraspMotor[6].Encoder->Init_Radian, Gr->GraspMotor[6].Encoder->Total_Radian);
        /*�ڻ�*/
        PID_DEAL(&Gr->GraspMotor[6].SPID, Gr->GraspMotor[6].PPID.Out, Gr->GraspMotor[6].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[6].SPID, Gr->GraspMotor[6].ExpSpeed, Gr->GraspMotor[6].Encoder->Speed[1]);
        Gr->GraspMotor[6].Encoder->Init_Radian = Gr->GraspMotor[6].Encoder->Total_Radian;
    }

    /**����**/
    /*�趨ֵ��Ϊ0�������ٶ�С��50(��ת)*/
    if((Gr->GraspMotor[3].Encoder->Speed[1] <= 100) && (Gr->GraspMotor[3].Encoder->Speed[1] >= -100) && (Gr->GraspMotor[3].ExpSpeed != 0))
    {
        clock ++;

        if(clock >= 50)
        {
            if(clock == 50)					//�����Ƕȳ�ʼ��
            {
                Gr->GraspMotor[3].Encoder->Lock_Radian = Gr->GraspMotor[3].Encoder->Total_Radian;
                lock = 2;

                if(Gr->GraspMotor[3].ExpSpeed > 0)									//�����ٶȷ����ж�
                    dire = 1;
                else
                    dire = -1;
            }

            if( (dire == 1 && Gr->GraspMotor[3].ExpSpeed < 0) || (dire == -1 && Gr->GraspMotor[3].ExpSpeed > 0) )								//����ֵ�෴��ֹͣ��ת����
            {
                clock = 0;																																																				//����ֹͣ����ת�ж�������0
                lock = 1;
            }
        }
    }

    /*�趨ֵΪ0������ֹͣ*/
    if(lock == 2)
    {
        PID_DEAL(&Gr->GraspMotor[3].PPID, Gr->GraspMotor[3].Encoder->Lock_Radian, Gr->GraspMotor[3].Encoder->Total_Radian);	//�⻷
        PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].PPID.Out, Gr->GraspMotor[3].Encoder->Speed[1]);									//�ڻ�
    }
    /*�趨ֵ��Ϊ0����ʼ����*/
    else if(lock == 1)
    {
        PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].ExpSpeed, Gr->GraspMotor[3].Encoder->Speed[1]);
        Gr->GraspMotor[3].Encoder->Lock_Radian = Gr->GraspMotor[3].Encoder->Total_Radian;
    }

    /**��ת**/
    /*�⻷*/
    PID_DEAL(&Gr->GraspMotor[4].PPID, Gr->GraspMotor[4].ExpRadian, Gr->GraspMotor[4].Encoder->Total_Radian);
    /*�ڻ�*/
    PID_DEAL(&Gr->GraspMotor[4].SPID, Gr->GraspMotor[4].PPID.Out, Gr->GraspMotor[4].Encoder->Speed[1]);

    /***************************����**************************/
    if(Gr->GraspMotor[5].ExpSpeed == 0)
    {
        /*�⻷*/
        PID_DEAL(&Gr->GraspMotor[5].PPID, Gr->GraspMotor[5].Encoder->Init_Radian, Gr->GraspMotor[5].Encoder->Total_Radian);
        /*�ڻ�*/
        PID_DEAL(&Gr->GraspMotor[5].SPID, Gr->GraspMotor[5].PPID.Out, Gr->GraspMotor[5].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[5].SPID, Gr->GraspMotor[5].ExpSpeed, Gr->GraspMotor[5].Encoder->Speed[1]);
        Gr->GraspMotor[5].Encoder->Init_Radian = Gr->GraspMotor[5].Encoder->Total_Radian;
    }


    if(Gr->GraspMotor[7].ExpSpeed == 0)
    {
        /*�⻷*/
        PID_DEAL(&Gr->GraspMotor[7].PPID, Gr->GraspMotor[7].Encoder->Init_Radian, Gr->GraspMotor[7].Encoder->Total_Radian);
        /*�ڻ�*/
        PID_DEAL(&Gr->GraspMotor[7].SPID, Gr->GraspMotor[7].PPID.Out, Gr->GraspMotor[7].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[7].SPID, Gr->GraspMotor[7].ExpSpeed, Gr->GraspMotor[7].Encoder->Speed[1]);
        Gr->GraspMotor[7].Encoder->Init_Radian = Gr->GraspMotor[7].Encoder->Total_Radian;
    }

    Gr->Can_Send_Grasp_1(MotorOutput_201_204);
    Gr->Can_Send_Grasp_2(MotorOutput_205_208);


//    //̧��5.18 ����������
//		if( rc->ch1 >= 500)
//		{
////				Gr->GraspMotor[0].state = DisFinish;
//				lift(&Gr->GraspMotor[0], 40 , 1 , -910);
//		}
//		if( rc->ch1 <= -500)
//		{
////				Gr->GraspMotor[0].state = DisFinish;
//				lift(&Gr->GraspMotor[0], 40 , 2 , -910);
//		}
//    //����+ƽ�� 5.18����ͨ��
//    if( rc->ch0 >= 500)
//    {
////				Gr->GraspMotor[2].state = DisFinish;
////				Gr->GraspMotor[1].state = DisFinish;
////				Telescoping(&Gr->GraspMotor[2],1);
//          Telescoping(&Gr->GraspMotor[1], 1);
//    }

//    if( rc->ch0 <= -500)
//    {
////				Gr->GraspMotor[2].state = DisFinish;
////				Gr->GraspMotor[1].state = DisFinish;
////				Telescoping(&Gr->GraspMotor[2],2);
//        Telescoping(&Gr->GraspMotor[1], 2);
//    }

//    //��ת5.18����ͨ��
//    if( rc->sw >= 500)
//    {
//        flip2(&Gr->GraspMotor[4], 155, 10, 1);
//    }

//    if( rc->sw <= -500)
//    {
//        flip2(&Gr->GraspMotor[4], 0, 10, 2);
//    }

//    pid_Cala(Gr);


}

/**
 * @description: ��λ��ȡ
 * @param {Gr_t} *Gr
 * @return {*}
 * @note 201-̧��	202-ƽ��	203-����	204-����	205-��ת	206-��ת	207-����
				��ʱ���ԣ�ʵ���Ͽ��ܲ���ȡ����λ��ֱ�ӷ�0
	wingchi:���ٲ��,��bug
 */
void Reset_Grasp(Gr_t *Gr)
{
    Gr->GraspMotor[0].state = Finish;
    Gr->GraspMotor[1].state = Finish;
    Gr->GraspMotor[2].state = DisFinish;
    Gr->GraspMotor[3].state = Finish;
    Gr->GraspMotor[4].state = Finish;
	
		if(Gr->GraspMotor[2].state == DisFinish)
		{
				Telescoping(&Gr->GraspMotor[2], 2);
		}
	
		if(Gr->GraspMotor[2].state == Finish)
		{
				Gr->GraspMotor[0].state = DisFinish;
				Gr->GraspMotor[1].state = DisFinish;
				Gr->GraspMotor[3].state = DisFinish;
				Gr->GraspMotor[4].state = DisFinish;
		}
		if(Gr->GraspMotor[0].state == DisFinish)
		{
			uplift(&Gr->GraspMotor[0], Gr->vL53L0, 2.0f, 2);
		}

		if(Gr->GraspMotor[1].state == DisFinish)
		{
			Translation(&Gr->GraspMotor[1], 2);//Ԥ��ƽ��
		}
		
		if(Gr->GraspMotor[3].state == DisFinish)
		{
			clip(&Gr->GraspMotor[3], Cilp_Speed, 2);
		}
		
		if(Gr->GraspMotor[4].state == DisFinish)
		{
			flip2(&Gr->GraspMotor[4], 0, 10, 2);//��ת����ʼ
		}

}

void bullet_Supply (Gr_t *Gr, Motor_t *Supply, int8_t dire)
{
//    static int8_t last_dire = 0;
//    static uint8_t clock = 0 ;
//    static uint8_t  lock = 1 ;

//    if( (dire != -1) && (dire != 1)) return ;

//    if(last_dire == 0)
//    {
//        last_dire = dire ; //��һ�θ�ֵ
//    }

//    if(last_dire != dire)
//    {
//        clock = 0;
//        lock = 1 ; // ����
//        last_dire = dire ;
//    }

//    Supply->ExpSpeed = Supply_Speed * dire ;

//    /*�������*/
//    if(int16_t_abs(Supply->Encoder->Speed[1]) <= 30)
//    {
//        clock ++ ;

//        if(clock > 20)
//        {
//            lock = 2 ; //����
//        }
//    }

//    if(lock == 2)
//        Supply->SPID.Out = 0 ;

//    else if(lock == 1 )
//    {
//        PID_DEAL(&Supply->SPID, Supply->ExpSpeed, Supply->Encoder->Speed[1]) ;

//    }

//    Gr->Can_Send_Grasp_2(MotorOutput_205_208);
}



/**
 * @description: ��ȡ�����ϵ�
 * @param {*}
 * @return {*}
 */
void Poweroff_Ctrl(Gr_t *Gr)
{
//    Gr->GraspMotor[0].Encoder->Init_Radian = Gr->GraspMotor[0].Encoder->Total_Radian;
//    Gr->GraspMotor[1].Encoder->Init_Radian = Gr->GraspMotor[1].Encoder->Total_Radian;
//    Gr->GraspMotor[2].Encoder->Init_Radian = Gr->GraspMotor[2].Encoder->Total_Radian;
//    Gr->GraspMotor[3].Encoder->Init_Radian = Gr->GraspMotor[3].Encoder->Total_Radian;
//    Gr->Can_Send_Grasp_1(0, 0, 0, 0);
//    Gr->Can_Send_Grasp_2(0, 0, 0, 0);
}




/**
 * @description: ���㺯��
 * @param {Gr_t} *Gr
 * @return {*}
 */
void pid_Cala(Gr_t *Gr)
{
    u8 i = 0;

		if (Gr->GraspMotor[0].state == DisFinish) //δ������ٶȻ�
		{
				PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].ExpSpeed, Gr->GraspMotor[0].Encoder->Speed[1]);
		}
		else if (Gr->GraspMotor[0].state == Finish) //������λ�û�
		{
				PID_DEAL(&Gr->GraspMotor[0].PPID, Gr->GraspMotor[0].Encoder->Lock_Radian, Gr->GraspMotor[0].Encoder->Total_Radian); //�⻷
				PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].PPID.Out, Gr->GraspMotor[0].Encoder->Speed[1]);					//�ڻ�
		}
		else
		{
				Gr->GraspMotor[0].SPID.Out = 0;
		}
	
    for (i = 1; i < 3; i++)
    {
        if (Gr->GraspMotor[i].state == DisFinish) //δ������ٶȻ�
        {
            PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].ExpSpeed, Gr->GraspMotor[i].Encoder->Speed[1]);
        }
        else if (Gr->GraspMotor[i].state == Finish) //������λ�û�
        {
            PID_DEAL(&Gr->GraspMotor[i].PPID, Gr->GraspMotor[i].Encoder->Lock_Radian, Gr->GraspMotor[i].Encoder->Radian); //�⻷
            PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].PPID.Out, Gr->GraspMotor[i].Encoder->Speed[1]);					//�ڻ�
        }
        else
        {
            Gr->GraspMotor[i].SPID.Out = 0;
        }
    }

		
        if (Gr->GraspMotor[3].state == DisFinish) //δ������ٶȻ�
        {
            PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].ExpSpeed, Gr->GraspMotor[3].Encoder->Speed[1]);
        }
        else if (Gr->GraspMotor[3].state == Finish) //������λ�û�
        {
            PID_DEAL(&Gr->GraspMotor[3].PPID, Gr->GraspMotor[3].Encoder->Lock_Radian, Gr->GraspMotor[3].Encoder->Radian); //�⻷
            PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].PPID.Out, Gr->GraspMotor[3].Encoder->Speed[1]);					//�ڻ�
        }
        else
        {
            Gr->GraspMotor[3].SPID.Out = 0;
        }
//    PID_DEAL(&Gr->GraspMotor[3].PPID, Gr->GraspMotor[3].ExpRadian, Gr->GraspMotor[3].Encoder->Radian);
//    PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].PPID.Out, Gr->GraspMotor[3].Encoder->Speed[1]);

    PID_DEAL(&Gr->GraspMotor[4].PPID, Gr->GraspMotor[4].ExpRadian, Gr->GraspMotor[4].Encoder->Total_Radian);
    PID_DEAL(&Gr->GraspMotor[4].SPID, Gr->GraspMotor[4].PPID.Out, Gr->GraspMotor[4].Encoder->Speed[1]);

    Gr->Can_Send_Grasp_1(MotorOutput_201_204);
    Gr->Can_Send_Grasp_2(MotorOutput_205_208);
		
    #if 0
    u8 i = 0;

    PID_DEAL(&Gr->GraspMotor[0].PPID, Gr->GraspMotor[0].ExpRadian, Gr->GraspMotor[0].Encoder->Total_Radian);
    PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].PPID.Out, Gr->GraspMotor[0].Encoder->Speed[1]);

    for (i = 1; i < 3; i++)
    {
        if (Gr->GraspMotor[i].state == DisFinish) //δ������ٶȻ�
        {
            PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].ExpSpeed, Gr->GraspMotor[i].Encoder->Speed[1]);
        }
        else if (Gr->GraspMotor[i].state == Finish) //������λ�û�
        {
            PID_DEAL(&Gr->GraspMotor[i].PPID, Gr->GraspMotor[i].Encoder->Lock_Radian, Gr->GraspMotor[i].Encoder->Radian); //�⻷
            PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].PPID.Out, Gr->GraspMotor[i].Encoder->Speed[1]);					//�ڻ�
        }
        else
        {
            Gr->GraspMotor[i].SPID.Out = 0;
        }
    }

    PID_DEAL(&Gr->GraspMotor[3].PPID, Gr->GraspMotor[3].ExpRadian, Gr->GraspMotor[3].Encoder->Total_Radian);
    PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].PPID.Out, Gr->GraspMotor[3].Encoder->Speed[1]);

    PID_DEAL(&Gr->GraspMotor[4].PPID, Gr->GraspMotor[4].ExpRadian, Gr->GraspMotor[4].Encoder->Total_Radian);
    PID_DEAL(&Gr->GraspMotor[4].SPID, Gr->GraspMotor[4].PPID.Out, Gr->GraspMotor[4].Encoder->Speed[1]);

    //  ��ת
    if(Gr->GraspMotor[7].state == DisFinish )
    {
        PID_DEAL(&Gr->GraspMotor[7].SPID, Gr->GraspMotor[7].ExpSpeed, Gr->GraspMotor[7].Encoder->Speed[1]);
    }
    else if (Gr->GraspMotor[7].state == Finish )
    {
        PID_DEAL(&Gr->GraspMotor[i].PPID, Gr->GraspMotor[i].Encoder->Lock_Radian, Gr->GraspMotor[i].Encoder->Total_Radian); //�⻷
        PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].PPID.Out, Gr->GraspMotor[i].Encoder->Speed[1]);					//�ڻ�
    }
    else
    {
        Gr->GraspMotor[7].SPID.Out = 0;
    }

    Gr->Can_Send_Grasp_1(MotorOutput_201_204);
    Gr->Can_Send_Grasp_2(MotorOutput_205_208);
    #endif
}


