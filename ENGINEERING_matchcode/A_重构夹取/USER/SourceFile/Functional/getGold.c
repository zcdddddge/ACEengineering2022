#include "getGold.h"

#if 0 
static u8 gold_lock = 1 ;
void change_Gold(Gr_t *Gr)
{

    if(gold_lock == 1)
    {
        gold_lock = 2;
        Gr->GraspMotor[0].state = DisFinish ;
    }
    else if (gold_lock == 2)
    {
        goldExchange(Gr) ;
        pid_Cala(Gr);
    }


}

/**
 * @description: 矿石自动换取金币
 * @param {*}
 * @return {*}
 * @note : 0抬升--1夹紧--前伸-前翻-旋转
 *         2.松夹-后伸-后翻-放下
 * 4.12 测试结果：使用旧工程从6开始不正常，新工程未测试，建议先看懂夹取一箱再改代码       （抬升 前伸    松夹   后伸 夹紧 前伸  ）  （ 松夹    后翻 夹紧 前翻    松夹   后伸 夹紧 前伸  ）  （ 松夹    前移 后翻 夹紧 前翻    松夹   后伸 夹紧 前伸）
 *																																													                                                        夹紧 前翻    松夹   后伸 夹紧	前伸																																																																																		
 *																																																																																																		 前移	后翻 夹紧	前翻 		松夹   后伸 夹紧 前伸																																			
*/																																																																																																							 																													
static void goldExchange(Gr_t *Gr)
{

    switch(Gr->state[2])
    {
        //抬升
        case  0 :
        {
            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 10.0f, 1);

            if (Gr->GraspMotor[0].state == Finish)
            {
                CLIP.state = DisFinish;  //夹子准备
                Gr->state[2] = 1;
            }

            break;
        }

        //夹紧
        case 1:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 1);

            if(Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[1].state = DisFinish;
                Gr->state[2] = 2;
            }

            break;
        }

        //前伸
        case 2 :
        {

            Telescoping(&Gr->GraspMotor[2], 2) ;

            if (TELESCOPING.state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;    //翻转准备
                Gr->state[2] = 3;
            }
        }

        //前翻
        case 3:
        {
            flip(&Gr->GraspMotor[4], &Gr->GraspMotor[5], 182.0f, 6.0f, 1); // 180-->182

            if (Gr->GraspMotor[4].state == Finish && Gr->GraspMotor[5].state == Finish)
            {
                CLIP.state = DisFinish ;
                CLIP.ExpSpeed = 0 ;
                Gr->GraspMotor[7].ExpSpeed = 0;
                Gr->GraspMotor[7].state = DisFinish;  //旋转准备
                Gr->state[2] = 5;
            }

            break;
        }

        //松夹
        case 5:
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 2);

            if(CLIP.state == Finish)
            {
                //Gr->GraspMotor[4].state=DisFinish ;
                //Gr->GraspMotor[5].state=DisFinish ;
                Gr->state[2]++;

            }
        }

        #if  0

        case 6:
        {
            flip(&Gr->GraspMotor[4], &Gr->GraspMotor[5], -160.0f, 20.0f, 2);

            if (FLIP_1.state == Finish && FLIP_2.state == Finish)
            {
                //UPLIFT.state= DisFinish;
                Gr->state[2]++ ;
            }

        }

        case 7:
        {
            Telescoping(&Gr->GraspMotor[2], 1);

            if(TELESCOPING.state == Finish)
            {
                FLIP_1.state = DisFinish;
                FLIP_2.state = DisFinish;
                Gr->state[2] ++ ;
            }
        }

        // 放下
        case 8 :
        {

            uplift(&Gr->GraspMotor[0], Gr->vL53L0, 1.0f, 2);

            if(UPLIFT.state == Finish)
            {
                FLIP_1.state = DisFinish;
                FLIP_2.state = DisFinish;
                CLIP.state         = DisFinish ;
                TELESCOPING.state  = DisFinish ;
                Gr->state[2] =  0 ;
                gold_lock = 1; // 解锁
            }
        }

        #endif

        default :
            break;


    }
}









/**
 * @description: 夹取地上的矿石,逻辑上使用了两把锁
 * @param {Gr_t} *Gr
 * @return {*}
 * @note 4.12 未测试，逻辑简单，可先测试
 */
void pick_Gold(Gr_t *Gr)
{
    switch( Gr->state[3] )
    {

        case 0 :
        {
            // 前翻
            flip2(&FLIP_1, 237.0, 6, 1) ;

            if(FLIP_1.state == Finish)
            {
                CLIP.state = DisFinish;
                Gr->state[3] = 1 ;
            }

            break;
        }

        //夹紧
        case 1 :
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 1);

            if (Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;
                Gr->state[2] = 2;
            }

            break;
        }

        //后翻
        case 2:
        {
            flip2(&Gr->GraspMotor[4], -160.0f, 20.0f, 2);

            if (FLIP_1.state == Finish)
            {
                CLIP.state = DisFinish;
                Gr->state[3]++;
            }

            break ;
        }

        // 松夹
        case 3 :
        {
            clip(&Gr->GraspMotor[3], Cilp_Speed, 2);

            if (Gr->GraspMotor[3].state == Finish)
            {
                Gr->GraspMotor[4].state = DisFinish;
                Gr->state[3] ++ ;
            }

            break;
        }

        case 4:
        {
            // 解锁
            CLIP.state = DisFinish;
            FLIP_1.state = DisFinish;

            Gr->state[3]++;  // 5标记已完成
        }

        default :
        {
            break;
        }
    }

}

#endif

/*================================================================================================
------------------------------------------以下为新的代码------------------------------------------
================================================================================================*/



/**
 * @description: 矿石自动换取金币
 * @param {*}
 * @return {*}
 * @note :
 * 
 *        （抬升 前伸    松夹   后伸 夹紧 前伸  ）  （ 松夹    后翻 夹紧 前翻    松夹   后伸 夹紧 前伸  ）  （ 松夹    前移 后翻 夹紧 前翻    松夹   后伸 夹紧 前伸）
 *																																	夹紧 前翻    松夹   后伸 夹紧	前伸																																																																																		
 *																																																										 前移	后翻 夹紧	前翻 		松夹   后伸 夹紧 前伸																																			
*/

void conversion_init(Gr_t *Gr)
{
    switch(Gr->conversion_state)
    {
			
			case 0 :
			{
					Gr->GraspMotor[0].state = DisFinish;//预备抬升
				
					uplift(&Gr->GraspMotor[0], Gr->vL53L0, 12.8f, 1);
				
					if(Gr->GraspMotor[0].state == Finish)
					{
							Gr->GraspMotor[2].state = DisFinish;//预备平移
							Gr->conversion_state = 1;
					}
			}
			
			case 1 :
			{
					Telescoping(&Gr->GraspMotor[2], 1);
				
					
			}
			
        default:
            break;
		}
}













