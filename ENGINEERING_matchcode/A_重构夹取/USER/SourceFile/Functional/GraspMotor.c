/*
 * @file: GraspMotor.c
 * @description: 定义了夹取电机初始化,遥控控制,复位,供弹函数
 * @date:
 * @history:
 */


#include "GraspMotor.h"

/**
 * @description: 夹取主部分初始化
 * @param {Gr_t} *Gr
 * @return {*}
 * @note 201-抬升	202-归一	203-伸缩	204-夹子	205-翻转	206-对位旋转	207-弹仓  208-旋转
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

    /*函数映射*/
    Gr->Can_Send_Grasp_1			= CAN1_201_To_204_SEND;
    Gr->Can_Send_Grasp_2	 		= CAN1_205_To_208_SEND;
    Gr->Get_Encoder						=	Return_Can1_201_208_Encoder;
		Gr->Get_Encoder2						=	Return_Can2_201_208_Encoder;
    Gr->Get_Sensor_t					= Return_Sensor_t;
    Gr->Get_VL53L0_t					= Return_VL53L0_t;

    for(i = 0; i < 8; i ++)
    {
        /*清零处理*/
        MotorValZero(&Gr->GraspMotor[i]);
        /*ID赋值*/
        Gr->GraspMotor[i].ID = i + 1;
        /*码盘赋值*/
        Gr->GraspMotor[i].Encoder = Gr->Get_Encoder(i + 1);
        PID_INIT(&Gr->GraspMotor[i].PPID, Ppid[i][0], Ppid[i][1], Ppid[i][2], 0, 8500);							//位置环初始化
        PID_INIT(&Gr->GraspMotor[i].SPID, Spid[i][0], Spid[i][1], Spid[i][2], 0, 16000);							//速度环初始化
        Gr->GraspMotor[i].ExpRadian							= Gr->GraspMotor[i].Encoder->Radian;						//初始化期望角度
        Gr->GraspMotor[i].Encoder->Init_Radian 	= Gr->GraspMotor[i].Encoder->Radian;    				//初始码盘值赋值
        Gr->GraspMotor[i].Encoder->Lock_Radian 	= Gr->GraspMotor[i].Encoder->Radian;	    			//初始化上锁角度
        Gr->GraspMotor[i].Encoder->Total_Radian = Gr->GraspMotor[i].Encoder->Radian;	    			//初始化总角度
        Gr->GraspMotor[i].Radio = 19;																														//初始化夹取电机减速比
        Gr->GraspMotor[i].ResetFlag = DisFinish ;
        Gr->GraspMotor[i].debug = 0;
    }

    PID_INIT(&Gr->GraspMotor[2].PPID, Ppid[2][0], Ppid[2][1], Ppid[2][2], 0, 14700);							//位置环初始化
		PID_INIT(&Gr->GraspMotor[3].SPID, Ppid[3][0], Ppid[3][1], Ppid[3][2], 0, 16000);
    PID_INIT(&Gr->GraspMotor[6].PPID, Ppid[6][0], Ppid[6][1], Ppid[6][2], 0, 14700);							//位置环初始化

    Gr->GraspMotor[0].Radio = 27;																														//初始化抬升电机减速比
    Gr->GraspMotor[2].Radio = 36;																														//初始化伸缩电机减速比
    Gr->GraspMotor[3].Radio = 19;																														//初始化夹取电机减速比
    Gr->GraspMotor[6].Radio = 36;																														//初始化补弹电机减速比
    Gr->GraspMotor[7].Radio = 36;


    /*状态值初始化*/
    Gr->state[0] = Gr->state[1] = Gr->state[2] = Gr->state[3] = DisFinish;
		Gr->conversion_state = DisFinish;
		
    /*复位值初始化*/
    Gr->reset[0] = Gr->reset[1] = Gr->reset[2] = Gr->reset[3] \
                                  = Gr->reset[4] = Gr->reset[5] = Gr->reset[6] = DisFinish;

    /*传感器初始化*/
    Gr->sensor	= Gr->Get_Sensor_t();
    Gr->sensor->Smooth_L = Gr->sensor->Smooth_R = Gr->sensor->Stretch_F = 2;

    /*VL53L0测距初始化*/
    Gr->vL53L0 = Gr->Get_VL53L0_t();
		
		
}



/**
 * @description: 遥控控制
 * @param {Gr_t} *Gr
 * @param {RC_ctrl_t} *r
 * @return {*}
 * @note 201-抬升	202-对位归一	203-伸缩	204-夹子	205-翻转	206-对位归一	207-弹仓
 *	 0 抬升   2伸缩   3 夹紧  4 翻转  7旋转
 */
void RC_Ctrl(Gr_t *Gr, RC_ctrl_t *rc)
{
    static int16_t clock = 0;
    static int8_t dire = 0;
    static int8_t lock = 1;


    Gr->GraspMotor[0].ExpSpeed   = -((float)(rc->ch1) * 10.0f); // 抬升
    Gr->GraspMotor[1].ExpSpeed   = -((float)(rc->ch0) * 15.0f); // 平移
    Gr->GraspMotor[2].ExpSpeed   = ((float)(rc->ch3) * 10.0f); // 伸缩
    Gr->GraspMotor[3].ExpSpeed   = -((float)(rc->ch2) * 10.0f);  // 夹紧
    Gr->GraspMotor[4].ExpRadian += ((float)(rc->sw) / 1600.0f); // 翻转
    //Gr->GraspMotor[6].ExpSpeed   = ((float)(rc->ch1) * 10.0f);// 弹仓
    //Gr->GraspMotor[7].ExpSpeed   = ((float)(rc->ch3) * 10.0f);  // 救援爪





    /****************************抬升***************************/
    if(Gr->GraspMotor[0].ExpSpeed == 0)
    {
        /*外环*/
        PID_DEAL(&Gr->GraspMotor[0].PPID, Gr->GraspMotor[0].Encoder->Init_Radian, Gr->GraspMotor[0].Encoder->Total_Radian);
        /*内环*/
        PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].PPID.Out, Gr->GraspMotor[0].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].ExpSpeed, Gr->GraspMotor[0].Encoder->Speed[1]);
        Gr->GraspMotor[0].Encoder->Init_Radian = Gr->GraspMotor[0].Encoder->Total_Radian;
    }

    /***************************对位**************************/
    if(Gr->GraspMotor[1].ExpSpeed == 0)
    {
        /*外环*/
        PID_DEAL(&Gr->GraspMotor[1].PPID, Gr->GraspMotor[1].Encoder->Init_Radian, Gr->GraspMotor[1].Encoder->Total_Radian);
        /*内环*/
        PID_DEAL(&Gr->GraspMotor[1].SPID, Gr->GraspMotor[1].PPID.Out, Gr->GraspMotor[1].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[1].SPID, Gr->GraspMotor[1].ExpSpeed, Gr->GraspMotor[1].Encoder->Speed[1]);
        Gr->GraspMotor[1].Encoder->Init_Radian = Gr->GraspMotor[1].Encoder->Total_Radian;
    }

    /***************************伸缩**************************/
    if(Gr->GraspMotor[2].ExpSpeed == 0)
    {
        /*外环*/
        PID_DEAL(&Gr->GraspMotor[2].PPID, Gr->GraspMotor[2].Encoder->Init_Radian, Gr->GraspMotor[2].Encoder->Total_Radian);
        /*内环*/
        PID_DEAL(&Gr->GraspMotor[2].SPID, Gr->GraspMotor[2].PPID.Out, Gr->GraspMotor[2].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[2].SPID, Gr->GraspMotor[2].ExpSpeed, Gr->GraspMotor[2].Encoder->Speed[1]);
        Gr->GraspMotor[2].Encoder->Init_Radian = Gr->GraspMotor[2].Encoder->Total_Radian;
    }

    /************************弹仓*******************************/
    if(Gr->GraspMotor[6].ExpSpeed == 0)
    {
        /*外环*/
        PID_DEAL(&Gr->GraspMotor[6].PPID, Gr->GraspMotor[6].Encoder->Init_Radian, Gr->GraspMotor[6].Encoder->Total_Radian);
        /*内环*/
        PID_DEAL(&Gr->GraspMotor[6].SPID, Gr->GraspMotor[6].PPID.Out, Gr->GraspMotor[6].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[6].SPID, Gr->GraspMotor[6].ExpSpeed, Gr->GraspMotor[6].Encoder->Speed[1]);
        Gr->GraspMotor[6].Encoder->Init_Radian = Gr->GraspMotor[6].Encoder->Total_Radian;
    }

    /**夹子**/
    /*设定值不为0，但是速度小于50(堵转)*/
    if((Gr->GraspMotor[3].Encoder->Speed[1] <= 100) && (Gr->GraspMotor[3].Encoder->Speed[1] >= -100) && (Gr->GraspMotor[3].ExpSpeed != 0))
    {
        clock ++;

        if(clock >= 50)
        {
            if(clock == 50)					//上锁角度初始化
            {
                Gr->GraspMotor[3].Encoder->Lock_Radian = Gr->GraspMotor[3].Encoder->Total_Radian;
                lock = 2;

                if(Gr->GraspMotor[3].ExpSpeed > 0)									//期望速度方向判断
                    dire = 1;
                else
                    dire = -1;
            }

            if( (dire == 1 && Gr->GraspMotor[3].ExpSpeed < 0) || (dire == -1 && Gr->GraspMotor[3].ExpSpeed > 0) )								//期望值相反，停止堵转处理
            {
                clock = 0;																																																				//主动停止，堵转判断心跳归0
                lock = 1;
            }
        }
    }

    /*设定值为0，主动停止*/
    if(lock == 2)
    {
        PID_DEAL(&Gr->GraspMotor[3].PPID, Gr->GraspMotor[3].Encoder->Lock_Radian, Gr->GraspMotor[3].Encoder->Total_Radian);	//外环
        PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].PPID.Out, Gr->GraspMotor[3].Encoder->Speed[1]);									//内环
    }
    /*设定值不为0，开始运行*/
    else if(lock == 1)
    {
        PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].ExpSpeed, Gr->GraspMotor[3].Encoder->Speed[1]);
        Gr->GraspMotor[3].Encoder->Lock_Radian = Gr->GraspMotor[3].Encoder->Total_Radian;
    }

    /**翻转**/
    /*外环*/
    PID_DEAL(&Gr->GraspMotor[4].PPID, Gr->GraspMotor[4].ExpRadian, Gr->GraspMotor[4].Encoder->Total_Radian);
    /*内环*/
    PID_DEAL(&Gr->GraspMotor[4].SPID, Gr->GraspMotor[4].PPID.Out, Gr->GraspMotor[4].Encoder->Speed[1]);

    /***************************伸缩**************************/
    if(Gr->GraspMotor[5].ExpSpeed == 0)
    {
        /*外环*/
        PID_DEAL(&Gr->GraspMotor[5].PPID, Gr->GraspMotor[5].Encoder->Init_Radian, Gr->GraspMotor[5].Encoder->Total_Radian);
        /*内环*/
        PID_DEAL(&Gr->GraspMotor[5].SPID, Gr->GraspMotor[5].PPID.Out, Gr->GraspMotor[5].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[5].SPID, Gr->GraspMotor[5].ExpSpeed, Gr->GraspMotor[5].Encoder->Speed[1]);
        Gr->GraspMotor[5].Encoder->Init_Radian = Gr->GraspMotor[5].Encoder->Total_Radian;
    }


    if(Gr->GraspMotor[7].ExpSpeed == 0)
    {
        /*外环*/
        PID_DEAL(&Gr->GraspMotor[7].PPID, Gr->GraspMotor[7].Encoder->Init_Radian, Gr->GraspMotor[7].Encoder->Total_Radian);
        /*内环*/
        PID_DEAL(&Gr->GraspMotor[7].SPID, Gr->GraspMotor[7].PPID.Out, Gr->GraspMotor[7].Encoder->Speed[1]);
    }
    else
    {
        PID_DEAL(&Gr->GraspMotor[7].SPID, Gr->GraspMotor[7].ExpSpeed, Gr->GraspMotor[7].Encoder->Speed[1]);
        Gr->GraspMotor[7].Encoder->Init_Radian = Gr->GraspMotor[7].Encoder->Total_Radian;
    }

    Gr->Can_Send_Grasp_1(MotorOutput_201_204);
    Gr->Can_Send_Grasp_2(MotorOutput_205_208);


//    //抬升5.18 测试有问题
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
//    //伸缩+平移 5.18测试通过
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

//    //翻转5.18测试通过
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
 * @description: 复位夹取
 * @param {Gr_t} *Gr
 * @return {*}
 * @note 201-抬升	202-平移	203-伸缩	204-夹子	205-翻转	206-翻转	207-弹仓
				临时测试，实际上可能不可取：复位后直接发0
	wingchi:寒假测过,有bug
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
			Translation(&Gr->GraspMotor[1], 2);//预备平移
		}
		
		if(Gr->GraspMotor[3].state == DisFinish)
		{
			clip(&Gr->GraspMotor[3], Cilp_Speed, 2);
		}
		
		if(Gr->GraspMotor[4].state == DisFinish)
		{
			flip2(&Gr->GraspMotor[4], 0, 10, 2);//翻转至初始
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
//        last_dire = dire ; //第一次赋值
//    }

//    if(last_dire != dire)
//    {
//        clock = 0;
//        lock = 1 ; // 解锁
//        last_dire = dire ;
//    }

//    Supply->ExpSpeed = Supply_Speed * dire ;

//    /*结束标记*/
//    if(int16_t_abs(Supply->Encoder->Speed[1]) <= 30)
//    {
//        clock ++ ;

//        if(clock > 20)
//        {
//            lock = 2 ; //上锁
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
 * @description: 夹取机构断电
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
 * @description: 计算函数
 * @param {Gr_t} *Gr
 * @return {*}
 */
void pid_Cala(Gr_t *Gr)
{
    u8 i = 0;

		if (Gr->GraspMotor[0].state == DisFinish) //未完成跑速度环
		{
				PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].ExpSpeed, Gr->GraspMotor[0].Encoder->Speed[1]);
		}
		else if (Gr->GraspMotor[0].state == Finish) //否则跑位置环
		{
				PID_DEAL(&Gr->GraspMotor[0].PPID, Gr->GraspMotor[0].Encoder->Lock_Radian, Gr->GraspMotor[0].Encoder->Total_Radian); //外环
				PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].PPID.Out, Gr->GraspMotor[0].Encoder->Speed[1]);					//内环
		}
		else
		{
				Gr->GraspMotor[0].SPID.Out = 0;
		}
	
    for (i = 1; i < 3; i++)
    {
        if (Gr->GraspMotor[i].state == DisFinish) //未完成跑速度环
        {
            PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].ExpSpeed, Gr->GraspMotor[i].Encoder->Speed[1]);
        }
        else if (Gr->GraspMotor[i].state == Finish) //否则跑位置环
        {
            PID_DEAL(&Gr->GraspMotor[i].PPID, Gr->GraspMotor[i].Encoder->Lock_Radian, Gr->GraspMotor[i].Encoder->Radian); //外环
            PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].PPID.Out, Gr->GraspMotor[i].Encoder->Speed[1]);					//内环
        }
        else
        {
            Gr->GraspMotor[i].SPID.Out = 0;
        }
    }

		
        if (Gr->GraspMotor[3].state == DisFinish) //未完成跑速度环
        {
            PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].ExpSpeed, Gr->GraspMotor[3].Encoder->Speed[1]);
        }
        else if (Gr->GraspMotor[3].state == Finish) //否则跑位置环
        {
            PID_DEAL(&Gr->GraspMotor[3].PPID, Gr->GraspMotor[3].Encoder->Lock_Radian, Gr->GraspMotor[3].Encoder->Radian); //外环
            PID_DEAL(&Gr->GraspMotor[3].SPID, Gr->GraspMotor[3].PPID.Out, Gr->GraspMotor[3].Encoder->Speed[1]);					//内环
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
        if (Gr->GraspMotor[i].state == DisFinish) //未完成跑速度环
        {
            PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].ExpSpeed, Gr->GraspMotor[i].Encoder->Speed[1]);
        }
        else if (Gr->GraspMotor[i].state == Finish) //否则跑位置环
        {
            PID_DEAL(&Gr->GraspMotor[i].PPID, Gr->GraspMotor[i].Encoder->Lock_Radian, Gr->GraspMotor[i].Encoder->Radian); //外环
            PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].PPID.Out, Gr->GraspMotor[i].Encoder->Speed[1]);					//内环
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

    //  旋转
    if(Gr->GraspMotor[7].state == DisFinish )
    {
        PID_DEAL(&Gr->GraspMotor[7].SPID, Gr->GraspMotor[7].ExpSpeed, Gr->GraspMotor[7].Encoder->Speed[1]);
    }
    else if (Gr->GraspMotor[7].state == Finish )
    {
        PID_DEAL(&Gr->GraspMotor[i].PPID, Gr->GraspMotor[i].Encoder->Lock_Radian, Gr->GraspMotor[i].Encoder->Total_Radian); //外环
        PID_DEAL(&Gr->GraspMotor[i].SPID, Gr->GraspMotor[i].PPID.Out, Gr->GraspMotor[i].Encoder->Speed[1]);					//内环
    }
    else
    {
        Gr->GraspMotor[7].SPID.Out = 0;
    }

    Gr->Can_Send_Grasp_1(MotorOutput_201_204);
    Gr->Can_Send_Grasp_2(MotorOutput_205_208);
    #endif
}


