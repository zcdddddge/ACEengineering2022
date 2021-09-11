/*
 * @Date: 2021-02-24 11:38:12
 * @LastEditTime: 2021-04-12 20:11:20
 * @LastEditors: Please set LastEditors
 * @Description: 定义了工程自动夹取控制函数,重点在状态整理
 */

#include "RobotAction.h"


extern Sensor_t Sensor;

#if 0

/**
 * @description: 自动夹取控制流程,重点整理状态
 * @param {Gr_t} *Gr
 * @param {u8} box 箱子数
 * @return {*}
 * @note 201-抬升	202-平移	203-伸缩	204-夹子	205-翻转	206-翻转	207-弹仓
 * 	 0 抬升   2伸缩   3 夹紧  4 翻转  7旋转
 * @note ：新工程未测试  抬升要用新工程无测距模块的函数
 */
void Auto_Ctrl(Gr_t *Gr, u8 box)
{
    static u8 box_lock = 1;
    static u8 boxs = 0;

    /*箱子锁解锁，重新赋值箱子(防止操作手误操作)*/
    if(box_lock == 1)
    {
        boxs = box;
        box_lock = 2;

        if(boxs != 0)
            Gr->GraspMotor[0].state = DisFinish;//预备抬升
    }

    /*箱子数目为1*/
    if(boxs == 1)
    {
        if(Gr->state[0] != 4)
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 10.0f, 1);		//夹取前抬升准备

            if(UPLIFT.state == Finish)
            {
                Auto_One_Box(Gr);
            }
        }
        else if(Gr->state[0] == 4)
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 1.0f, 2);		//夹取后放回

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



    else if(boxs == 2)/*箱子数目为2*/
    {
        if(Gr->state[1] != 4)
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 13.5f, 1);		//夹取前抬升准备

            if(Gr->GraspMotor[0].state == Finish)
            {
                Auto_Two_Box(Gr);
            }
        }
        else if(Gr->state[1] == 4)
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 4.5f, 2);		//夹取后放回

            if(Gr->GraspMotor[0].state == Finish)
            {
                Gr->GraspMotor[1].ExpSpeed = 0;							//平移速度置0
                Gr->GraspMotor[1].state = DisFinish;				//平移标记为未完成
                Gr->GraspMotor[3].ExpSpeed = 0;
                Gr->GraspMotor[3].state = DisFinish;
                Gr->GraspMotor[4].state = DisFinish;
                Gr->GraspMotor[5].state = DisFinish;
                box_lock = 1;                             // 箱子解锁
                boxs = 0;
                Gr->state[1] = 0;
                //临时用于判断结束
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
 * @description: 自动夹取一箱
 * @param {*}
 * @return {*}
 */
static void Auto_One_Box(Gr_t *Gr)
{
    switch (Gr->state[0])
    {
        /*前翻*/
        case 0:
        {
            flip(&Gr->GraspMotor[4], &Gr->GraspMotor[5], 182.0f, 6.0f, 1);

            if(Gr->GraspMotor[4].state == Finish && Gr->GraspMotor[5].state == Finish)
            {
                Gr->GraspMotor[3].ExpSpeed = 0;
                Gr->GraspMotor[3].state = DisFinish;					//夹子标志为未完成
                Gr->state[0] = 1;
            }

            break;
        }

        /*夹紧*/
        case 1:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 1);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;					//翻转标记为未完成
                Gr->GraspMotor[5].state = DisFinish;
                Gr->state[0] = 2;
            }

            break;
        }

        /*后翻*/
        case 2:
        {
            flip(&Gr->GraspMotor[4], &Gr->GraspMotor[5], -160.0f, 20.0f, 2); // -170-->-160  变大

            if(Gr->GraspMotor[4].state == Finish && Gr->GraspMotor[5].state == Finish)
            {
                Gr->GraspMotor[3].ExpSpeed = 0;
                Gr->GraspMotor[3].state = DisFinish;					//夹子标志为未完成
                Gr->state[0] = 3;
            }

            break;
        }

        /*松夹*/
        case 3:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 2);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[3].ExpSpeed = 0;
                Gr->GraspMotor[3].state = DisFinish;				//夹子标记为未完成
                Gr->GraspMotor[0].state = DisFinish;				//抬升标记为未完成
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
        /*平移*/
        case 0:
        {
            rail(&Gr->GraspMotor[1], Gr->sensor, 1);

            if(Gr->GraspMotor[1].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;					//翻转标记为未完成
                Gr->GraspMotor[5].state = DisFinish;
                Gr->state[0] = 0;															//夹取一箱为重新开始
                Gr->state[1] = 1;
            }

            break;
        }

        /*夹取一箱*/
        case 1:
        {
            Auto_One_Box(Gr);

            if(Gr->state[0] == 4)
            {
                Gr->GraspMotor[0].state = Finish;						//抬升标记为完成
                Gr->GraspMotor[1].state = DisFinish;				//平移标记为未完成
                Gr->state[1] = 2;
            }

            break;
        }

        /*平移*/
        case 2:
        {
            rail(&Gr->GraspMotor[1], Gr->sensor, 1);

            if(Gr->GraspMotor[1].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;					//翻转标记为未完成
                Gr->GraspMotor[5].state = DisFinish;
                Gr->state[0] = 0;															//夹取一箱为重新开始
                Gr->state[1] = 3;
            }

            break;
        }

        /*夹取一箱*/
        case 3:
        {
            Auto_One_Box(Gr);

            if(Gr->state[0] == 4)
            {
                Gr->GraspMotor[0].state = DisFinish;						//抬升标记为完成
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
------------------------------------------以下为新的代码------------------------------------------
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
 * @description:夹取金矿初始化
 * @param {Gr_t} *Gr
 * @param
 * @return {*}
 * @note 201-抬升	202-平移	203-伸缩	204-夹子	205-翻转	206-翻转	207-弹仓
 * 	 0 抬升   2伸缩   3 夹紧  4 翻转  7旋转
 * @note ：此函数为向上抬与往前伸 抬升具体的高度还要再测
 */
static void Grasp_init(Gr_t *Gr)
{
    switch (Gr->state[0])
    {
        /*抬升*/
        case 0:
        {
            Gr->GraspMotor[0].state = DisFinish;//预备抬升

            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 10.0f, 1);

            if(Gr->GraspMotor[0].state == Finish)
            {
                Gr->GraspMotor[1].state = DisFinish;//预备平移
                Gr->GraspMotor[4].state = DisFinish;//预备夹取
                Gr->state[0] = 1;
            }

            break;
        }

        /*前移 翻转*/
        case 1:
        {
            Translation(&Gr->GraspMotor[1], 1);//平移
            if(Gr->GraspMotor[1].state == Finish)//
            {
                Gr->GraspMotor[2].state = DisFinish;//预备夹取
                Gr->state[0] = 2;
            }

            break;
        }

        /*前伸*/
        case 2:
        {

                Telescoping(&Gr->GraspMotor[2], 1);

                if(Gr->GraspMotor[2].state == Finish)
                {
										flip2(&Gr->GraspMotor[4], 170, 10, 1);//翻转至90°
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
		Gr->GraspMotor[0].state = DisFinish;//预备下降
	
		uplift(&Gr->GraspMotor[0], Gr->vL53L0, 1.0f, 2);
	
		if(Gr->GraspMotor[0].state == Finish)
		{
				Gr->GraspMotor[1].state = DisFinish;//预备平移
        Gr->GraspMotor[4].state = DisFinish;//预备夹取
		}
		
		if(Gr->GraspMotor[1].state == Finish)
		{
				Translation(&Gr->GraspMotor[1], 2);//预备平移
		}
		
		if(Gr->GraspMotor[4].state == Finish)
		{
				flip2(&Gr->GraspMotor[4], 0, 10, 2);//翻转至初始
		}
}


static void Grasp_First(Gr_t *Gr)
{
    switch (Gr->state[0])
    {
        /*前翻*/
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

        /*夹紧*/
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

        /*后翻*/
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

        /*后伸*/
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

        /*松夹*/
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

        /*后移 翻转*/
        case 8:
        {
            Translation(&Gr->GraspMotor[1], 2);//预备平移
            flip2(&Gr->GraspMotor[4], 65, 10, 1);//预备翻转至90°

            if(Gr->GraspMotor[1].state == Finish || Gr->GraspMotor[4].state == Finish)
            {
                Gr->state[0] = 9;//复位，等待下一条夹取指令
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
        /*初始化完成了才能进行夹取*/
        /*前伸*/
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

        /*前翻*/
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

        /*夹紧*/
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

        /*后伸*/
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

        /*后翻*/
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

        /*松夹*/
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

        /*翻转*/
        case 15:
        {
            flip2(&Gr->GraspMotor[4], 65, 10, 1);//预备翻转至90°

            if(Gr->GraspMotor[4].state == Finish)
            {
                Gr->state[0] = 16;//复位，等待下一条夹取指令
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
        /*初始化完成了才能进行夹取*/
        /*前伸*/
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

        /*前翻*/
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

        /*夹紧*/
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

        /*后伸*/
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
 * @description: 夹取失败复位:考虑到一般都是前翻时打到箱子-只执行后翻,并保持抬升,没有改变其他状态
 * @param {Gr_t} *Gr
 * @return {*}
 * @note
 * 		 4.12 忘记了测试效果，要重新测
 */
void Reset(Gr_t *Gr)
{

    flip2(&Gr->GraspMotor[4], 20.0f, 10, 2);

    if(Gr->GraspMotor[0].state == Finish)
    {
        PID_DEAL(&Gr->GraspMotor[0].PPID, Gr->GraspMotor[0].Encoder->Lock_Radian, Gr->GraspMotor[0].Encoder->Total_Radian);										//外环
        PID_DEAL(&Gr->GraspMotor[0].SPID, Gr->GraspMotor[0].PPID.Out, Gr->GraspMotor[0].Encoder->Speed[0]);
    }

    PID_DEAL(&Gr->GraspMotor[4].PPID, Gr->GraspMotor[4].ExpRadian, Gr->GraspMotor[4].Encoder->Total_Radian);
    PID_DEAL(&Gr->GraspMotor[5].PPID, Gr->GraspMotor[5].ExpRadian, Gr->GraspMotor[5].Encoder->Total_Radian);

    Gr->Can_Send_Grasp_1(MotorOutput_201_204);
    Gr->Can_Send_Grasp_2(MotorOutput_205_208);
}



