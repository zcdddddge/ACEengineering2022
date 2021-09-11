#include "BoardCommuni.h"
#include "CAN2_ISR.h"

RC_ctrl_t *rc;
/*板间通信缓冲区--B板到A板*/
u8 temp[8]; //uint8_t

/*板间通信缓冲区A板到B板*/
uint8_t *Can2_temp;

u8 Smooth_L;
u8 Smooth_R;

/*板间通信初始化*/
void BoardCommuni_Init(void)
{
    rc = Return_Remote_Point();
    Can2_temp = Return_CAN2_Board_Data();
}

void BoardCommuni_DataUpdate(int16_t *speed)
{

    if (Can2_temp[0] == 0xAC && Can2_temp[7] == 0xCE)
    {
        *speed = (int16_t)(Can2_temp[1] | (Can2_temp[2] << 8));
        Smooth_L = Can2_temp[3];
        Smooth_R = Can2_temp[4];
    }
    else
    {
        *speed = 0;
    }
}

/*新增--未测试*/
//void Send_KeyBoard_TO_Board(void)
//{
//    temp[0] = rc->KV.x;
//    temp[1] = (rc->KV.x >> 8);
//    temp[2] = rc->KV.y;
//    temp[3] = (rc->KV.y >> 8);
//    temp[4] = rc->KV.press_l;
//    temp[5] = rc->KV.kv0;
//    temp[6] = (rc->KV.kv0 >> 8);
//    temp[7] = 0x77;
//    CAN2_To_Board(temp, 0x0404);

//    temp[0] = rc->KV.kv1;
//    temp[1] = (rc->KV.kv1 >> 8);
//    temp[2] = 0;
//    temp[3] = 0;
//    temp[4] = 0;
//    temp[5] = 0;
//    temp[6] = 0xAC;
//    temp[7] = 0xCE;
//    CAN2_To_Board(temp, 0x0405);
//}

/*发送遥控数据*/
void Send_RC_To_Board(void)
{
    temp[0] = rc->ch0;        // ch0 低8位
    temp[1] = (rc->ch0 >> 8); //ch0 高8位
    temp[2] = rc->ch1;
    temp[3] = (rc->ch1 >> 8);
    temp[4] = rc->ch2;
    temp[5] = (rc->ch2 >> 8);
    temp[6] = rc->ch3;
    temp[7] = (rc->ch3 >> 8);
    CAN2_To_Board(temp, 0x0401);

    temp[0] = rc->sw;
    temp[1] = (rc->sw >> 8);
    temp[2] = rc->s1;
    temp[3] = rc->s2;
    temp[4] = rc->KV.key;
    temp[5] = (rc->KV.key >> 8);
    temp[6] = 0;
    temp[7] = 0;
    CAN2_To_Board(temp, 0x0402);
}

/*发送控制指令至板子*/
void Send_Ctrl_To_Board(unsigned char Grasp_Up, unsigned char Translation, unsigned char Telescoping, unsigned char Clap, unsigned char Flip)
{
    temp[0] = 0x0A;
    temp[1] = Grasp_Up;
    temp[2] = Translation;
    temp[3] = Telescoping;
    temp[4] = Clap;
    temp[5] = Flip;
    temp[6] = 0;
    temp[7] = 0xCE;
    CAN2_To_Board(temp, 0x0403);
	

}
