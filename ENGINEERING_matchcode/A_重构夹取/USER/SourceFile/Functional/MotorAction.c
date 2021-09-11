/*
 * @Date: 2021-02-24 11:34:45
 * @LastEditTime: 2021-04-12 20:03:10
 * @LastEditors: Please set LastEditors
 * @Description: 工程夹取单个电机的控制逻辑
 */

#include "MotorAction.h"
/**
  * @description: 使得电机转过绝对角度
  * @param {Motor_t} *filp1
  * @param {float} exp
  * @param {float} limit
  * @param {u8} dire 1-前翻  2-后翻
  * @return {*}
  */
/*测试版:3.27 测试通过*/
void flip2(Motor_t *filp1, float exp, float limit, u8 dire)
{
    static int16_t clock = 0;

    if (dire == 1)
    {

        filp1->ExpRadian -= 0.6f;

        if(filp1->ExpRadian <= filp1->Encoder->Init_Radian - exp)
        {
            //     x <= 6-182 =
            filp1->ExpRadian = filp1->Encoder->Init_Radian - exp;
        }

        /*完成标志*/
        // 平行： -151 <=18-exp  ===>exp = 169
        // 垂直： -225 + exp -6 <=6  ===> exp = 237
        if(float_abs(filp1->Encoder->Total_Radian + exp - filp1->Encoder->Init_Radian) <= limit)
        {
            filp1->state = Finish;
            filp1->ExpRadian = filp1->Encoder->Init_Radian - exp;
        }

    }
    else if (dire == 2)
    {
        filp1->ExpRadian += 0.4f;


        if(filp1->ExpRadian >= filp1->Encoder->Init_Radian - exp)
        {
            //     x <= 6-182 =
            filp1->ExpRadian = filp1->Encoder->Init_Radian - exp;
        }

        /*完成标志*/
        // 平行： -151 <=18-exp  ===>exp = 169
        // 垂直： -225 + exp -6 <=6  ===> exp = 237
        if(float_abs(filp1->Encoder->Total_Radian + exp - filp1->Encoder->Init_Radian) <= limit)
        {
            filp1->state = Finish;
            filp1->ExpRadian = filp1->Encoder->Init_Radian - exp;
        }
				else if(filp1->Encoder->Speed[1] <=30 || filp1->Encoder->Speed[1] >=30)
				{
						clock++;
            if(clock > 100)
            {
                clock = 0;
                filp1->ExpRadian = filp1->Encoder->Total_Radian;
                filp1->state = Finish;
            }
				}
    }
    else
    {
        filp1->ExpRadian = filp1->Encoder->Total_Radian;
    }
#if 1
    PID_DEAL(&filp1->PPID, filp1->ExpRadian, filp1->Encoder->Total_Radian);
    PID_DEAL(&filp1->SPID, filp1->PPID.Out, filp1->Encoder->Speed[1]);
		CAN1_205_To_208_SEND(filp1->SPID.Out,0,0,0);//filp1->SPID.Out
#endif
}

#if 1
/**
  * @description: 使得电机转过绝对角度
  * @param {Motor_t} *filp1
  * @param {Motor_t} *filp2
  * @param {float} exp
  * @param {float} limit
  * @param {u8} dire 1-前翻  2-后翻
  * @return {*}
  *	@note 没有两个电机了,用上面那个
  */
void flip(Motor_t *filp1, Motor_t *filp2, float exp, float limit, u8 dire)
{
    static int16_t clock[2] = {0, 0};

    if(dire == 1)
    {
        filp1->ExpRadian += 0.6f;
        filp2->ExpRadian -= 0.6f;

        /*限制*/
        if(filp1->ExpRadian >= filp1->Encoder->Init_Radian + exp)
        {
            filp1->ExpRadian = filp1->Encoder->Init_Radian + exp;
        }

        if(filp2->ExpRadian <= filp2->Encoder->Init_Radian - exp)
        {
            filp2->ExpRadian = filp2->Encoder->Init_Radian - exp;
        }

        /*完成标志*/
        if(float_abs(filp1->Encoder->Total_Radian - exp - filp1->Encoder->Init_Radian) <= limit)
        {
            filp1->state = Finish;
            filp1->ExpRadian = filp1->Encoder->Init_Radian - exp;
        }

        if(float_abs(filp2->Encoder->Total_Radian + exp - filp2->Encoder->Init_Radian) <= limit)
        {
            filp2->state = Finish;
            filp1->ExpRadian = filp1->Encoder->Init_Radian - exp;
        }
    }
    else if(dire == 2)
    {
        filp1->ExpRadian -= 0.4f;
        filp2->ExpRadian += 0.4f;

        /*完成标志*/
        if(float_abs(filp1->Encoder->Total_Radian - filp1->Encoder->Init_Radian) <= limit)
        {
            clock[0] ++;
        }

        if(float_abs(filp2->Encoder->Total_Radian - filp2->Encoder->Init_Radian) <= limit)
        {
            clock[1] ++;
        }

        if(clock[0] >= 20 && clock[1] >= 20)
        {
            filp1->ExpRadian = filp1->Encoder->Total_Radian;
            filp2->ExpRadian = filp2->Encoder->Total_Radian;
            filp1->state = Finish;
            filp2->state = Finish;
            clock[0] = 0;
            clock[1] = 0;
        }
    }
    else
    {
        filp1->ExpRadian = filp1->Encoder->Total_Radian;
        filp2->ExpRadian = filp2->Encoder->Total_Radian;
    }

}
#endif



/**
 * @description: 夹子
 * @param {Motor_t} *clip
 * @param {int16_t} exp  : expect Speed
 * @param {u8} dire      : 1 夹紧   2 松开
 * @return {*}
 */
void clip(Motor_t *clip, int16_t exp, u8 dire)
{
    static int16_t clock = 0;
    clip->debug ++ ;

    if(dire == 1 || dire == 2)
    {
        clip->ExpSpeed = (3 - dire * 2) * exp;

        /*结束判断*/
        if(int16_t_abs(clip->Encoder->Speed[1]) <= 2000)
        {
            clock ++;

            if(clock > 2)
            {
                clock = 0;
								clip->ExpSpeed = 0;
                clip->Encoder->Lock_Radian = clip->Encoder->Radian;
                clip->state = Finish;
            }
        }
    }
    else
    {
        clip->Encoder->Lock_Radian = clip->Encoder->Radian;
    }
#if 1
    if(clip->ExpSpeed == 0)
    {
			if(dire == 1)
			{
        /*外环*/
        PID_DEAL(&clip->PPID, clip->Encoder->Init_Radian, clip->Encoder->Total_Radian);
        /*内环*/
        PID_DEAL(&clip->SPID, clip->PPID.Out, clip->Encoder->Speed[1]);
			}
			if(dire == 2)
			{
					clip->SPID.Out =  0;
			}
    }
    else
    {
        PID_DEAL(&clip->SPID, clip->ExpSpeed, clip->Encoder->Speed[1]);
        clip->Encoder->Radian = clip->Encoder->Total_Radian;
    }	
#endif	
}


/**
 * @description: 夹子(新版 算法测试通过，等上车测试07.07)
  * @param {Motor_t} *filp1
  * @param {float} exp
  * @param {float} limit
  * @param {u8} dire 1-松夹  2-夹紧
 * @return {*}
 */
void clamp(Motor_t *clamp, float exp, u8 dire)
{
    static int16_t clock = 0;
    static int8_t last_dire = 0;
    if (last_dire == 0)
    {
        last_dire = dire;
    }

    if (last_dire != dire) //当前要求方向和上一次方向不同，状态值复位
    {
				clamp->state = DisFinish;
        clock = 0;
        last_dire = dire;
    }
    if (dire == 1)
    {
				clamp->SPID.Out = -4000;
			
        if(int16_t_abs(clamp->Encoder->Speed[1]) <= 50)
        {
            clock ++;

            if(clock > 60)
            {
                clock = 0;
								clamp->ExpSpeed = 0;
                clamp->Encoder->Lock_Radian = clamp->Encoder->Radian;
                clamp->state = Finish;
            }
					if(clamp->state == Finish)
					{
								PID_DEAL(&clamp->PPID, clamp->Encoder->Lock_Radian, clamp->Encoder->Radian); //外环
								PID_DEAL(&clamp->SPID, clamp->PPID.Out, clamp->Encoder->Speed[1]);					//内环
					}
        }
			
    }
    else if (dire == 2)
    {
				clamp->SPID.Out = 1000;
			
        if(int16_t_abs(clamp->Encoder->Speed[1]) <= 100)
        {
            clock ++;

            if(clock > 50)
            {
                clock = 0;
								clamp->ExpSpeed = 0;
                clamp->Encoder->Lock_Radian = clamp->Encoder->Radian;
                clamp->state = Finish;
            }
						if(clamp->state == Finish)
						{
								clamp->SPID.Out = 0;
						}
        }
    }

}



/**
 * @description: 抬升电机的位置环,未测试!!!
 * @param {Motor_t} *lift 电机
 * @param {float} limit   限制
 * @param {u8} dire       方向
 * @param {float} exp     期望角度 100 1 910
 * @note 最后可以封装为一个电机的位置环  未测试
 */
void lift (Motor_t *liftup, float limit, u8 dire, float exp )
{
    static int16_t clock = 0;

    if(dire == 1)
    {
        liftup->ExpRadian += 0.6f;

        if(float_abs(liftup->Encoder->Total_Radian - exp - liftup->Encoder->Init_Radian ) <= limit)
        {
            liftup->state = Finish ;
            liftup->ExpRadian = liftup->Encoder->Init_Radian - exp;
        }
    }
    else if(dire == 2)
    {
        liftup ->ExpRadian -= 0.4f;

        if (float_abs(liftup->Encoder->Total_Radian - liftup->Encoder->Init_Radian) <= limit)
        {
            clock++;
        }

        if(clock >= 20)
        {
            liftup->ExpRadian = liftup->Encoder->Total_Radian;
            liftup->state = Finish;
            clock = 0 ;
        }
    }
    else
    {
        liftup->ExpRadian = liftup->Encoder->Total_Radian ;
    }
}



/**
 * @description: 抬升,有传感器
 * @param {*Motor_t *uplift,u8 dire :1 抬升  2 放下}
 * @return {*}
 */
void uplift(Motor_t *uplift, VL53L0_t *vl53l0, float dis, u8 dire)
{
//		uplift->state = DisFinish ;
	static float last_dis = 0.0f;
	if(last_dis != dis)
	{
		uplift->state = DisFinish;
		last_dis = dis;
	}
	
	
    if(dire == 1)																													//向上
    {
        uplift->ExpSpeed = -UpLift_Speed; 																		//初速度赋值

        if(vl53l0->distance - vl53l0->InitDistance >= dis)//结束标志
        {
            uplift->state = Finish;
            uplift->Encoder->Lock_Radian = uplift->Encoder->Total_Radian;
        }
    }
    else if(dire == 2)																										//向下
    {
        uplift->ExpSpeed = UpLift_Speed*0.3f; //初速度赋值 测试

        if(vl53l0->distance - vl53l0->InitDistance <= dis)									//结束标志
        {
						uplift->state = Finish;
            uplift->Encoder->Lock_Radian = uplift->Encoder->Total_Radian;
        }
    }
    else
        uplift->ExpSpeed = 0;
		
		
		if(uplift->state == Finish)
		{
				PID_DEAL(&uplift->PPID, uplift->Encoder->Lock_Radian, uplift->Encoder->Total_Radian); //外环
				PID_DEAL(&uplift->SPID, uplift->PPID.Out, uplift->Encoder->Speed[1]);					//内环
		}
		else if(uplift->state == DisFinish)
		{
				PID_DEAL(&uplift->SPID, uplift->ExpSpeed, uplift->Encoder->Speed[1]);
		}
}




/**
 * @description: 伸缩
 * @param {Motor_t} *telescoping
 * @param {u8} dire
 * @return {*}
 * @note 靠堵转试试    2--后伸  1--前伸
 */
void Telescoping(Motor_t *telescoping, u8 dire)
{
    static uint8_t clock = 0 ;
    static int8_t lock = 1;
    static int8_t last_dire = 0;
    if (last_dire == 0)
    {
        last_dire = dire;
    }

    if (last_dire != dire) //当前要求方向和上一次方向不同，状态值复位
    {
        lock = 1;
				telescoping->state = DisFinish;
        clock = 0;
        last_dire = dire;
    }

    if(dire == 1 || dire == 2)
    {
        telescoping->ExpSpeed = (dire * 2 - 3) * Telescoping_Speed*0.5f;

        if (int16_t_abs(telescoping->Encoder->Speed[1]) <= 100)
        {
            clock++;

            if (clock > 5)
            {
                clock = 0;
								lock = 2;
                telescoping->Encoder->Lock_Radian = telescoping->Encoder->Radian;
                telescoping->state = Finish;
            }
        }
    }
    else
    {
        telescoping->Encoder->Lock_Radian = telescoping->Encoder->Radian;
    }
    if (telescoping->state == Finish)
    {
        PID_DEAL(&telescoping->PPID, telescoping->Encoder->Lock_Radian, telescoping->Encoder->Radian);
        PID_DEAL(&telescoping->SPID, telescoping->PPID.Out, telescoping->Encoder->Speed[1]);
//                telescoping->SPID.Out = 0; // 直接让他停下来
    }
    else if (telescoping->state == DisFinish)
    {
        PID_DEAL(&telescoping->SPID, telescoping->ExpSpeed, telescoping->Encoder->Speed[1]);
    }
}


/**
 * @description: 平移
 * @param {Motor_t} *telescoping
 * @param {u8} dire
 * @return {*}
 * @note 靠堵转试试    2--后伸  1--前伸
 */
void Translation(Motor_t *translation, u8 dire)
{
    static uint8_t clock = 0 ;
    static int8_t lock = 1;
    static int8_t last_dire = 0;
    if (last_dire == 0)
    {
        last_dire = dire;
    }

    if (last_dire != dire) //当前要求方向和上一次方向不同，状态值复位
    {
        lock = 1;
        clock = 0;
				translation->state = DisFinish;
        last_dire = dire;
    }
    if(dire == 1 || dire == 2)
    {
        translation->ExpSpeed = (dire * 2 - 3) * Telescoping_Speed;

        if (int16_t_abs(translation->Encoder->Speed[1]) <= 100)
        {
            clock++;

            if (clock > 5)
            {
                clock = 0;
								lock = 2;
                translation->Encoder->Lock_Radian = translation->Encoder->Radian;
                translation->state = Finish;
            }
        }
    }
    else
    {
        translation->Encoder->Lock_Radian = translation->Encoder->Radian;
    }
    if (translation->state == Finish)
    {
//        PID_DEAL(&telescoping->PPID, telescoping->Encoder->Lock_Radian, telescoping->Encoder->Radian);
//        PID_DEAL(&telescoping->SPID, telescoping->PPID.Out, telescoping->Encoder->Speed[1]);
                translation->SPID.Out = 0; // 直接让他停下来
    }
    else if (translation->state == DisFinish)
    {
        PID_DEAL(&translation->SPID, translation->ExpSpeed, translation->Encoder->Speed[1]);
    }
}


#if 0
/**
 * @description: 旋转电机,测试失败
 * @param {Motor_t} *rotate
 * @return {*}
 * @note 只跑速度环
 */
void rotate(Motor_t *rotate)
{
    rotate->ExpSpeed = Rotate_Speed;
    rotate->debug ++ ;
    rotate->ResetFlag++ ;

    if(rotate->ResetFlag >= 10)
    {
        rotate->state = Finish;
    }

}
#endif


/**
 * @description: 平移
 * @param {*}
 * @return {*}
 * @note 到时候电机确定了与伸缩同一套算法
 */

void rail(Motor_t *rall, Sensor_t *val, u8 dire)
{
    /*速度赋值*/
    if(dire == 1 || dire == 2)
    {
        rall->ExpSpeed = (dire * 2 - 3) * Rall_Speed;
    }
    else
    {
        rall->SPID.Out = 0;
    }

    /*完成标志*/
    if( (val->Smooth_L == 1) && (val->Smooth_R == 1) )
    {
        rall->state = Finish;
        rall->Encoder->Lock_Radian = rall->Encoder->Total_Radian;
    }
}


