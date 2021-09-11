 #include "BoardCommuni.h"
#include "CAN2_ISR.h"

Board_Communi_t Board_Communi;
uint8_t temp[8];

extern Sensor_t Sensor;

void Board_Communi_Init(void)
{
    Board_Communi.Rc = Return_CAN2_Board_Data();
}

/*发送控制指令到板子*/
void Send_Crtl_To_Board(void)
{
    temp[0] = 0xAC;
    temp[1] = (unsigned char)get_sensor_flag();
    temp[2] = 0;
    temp[3] = 0;
    temp[4] = 0;
    temp[5] = 0;
    temp[6] = 0;
    temp[7] = 0xCE;
    CAN2_To_Board(temp, 0x11);
}

void Board_Communi_Updata(void)
{
    static int16_t clock[2] = {0, 0};
    static uint8_t flag = 0;

    switch (Board_Communi.Rc[16])
    {
        case 1:
        {
            Board_Communi.Can_RC.ch0 = (int16_t)(Board_Communi.Rc[1] << 8) | (Board_Communi.Rc[0]); //ch0 合成
            Board_Communi.Can_RC.ch1 = (int16_t)(Board_Communi.Rc[3] << 8) | (Board_Communi.Rc[2]);
            Board_Communi.Can_RC.ch2 = (int16_t)(Board_Communi.Rc[5] << 8) | (Board_Communi.Rc[4]);
            Board_Communi.Can_RC.ch3 = (int16_t)(Board_Communi.Rc[7] << 8) | (Board_Communi.Rc[6]);
            Board_Communi.Rc[16] = 0;
            clock[0] = 0;
            break;
        }

        case 0:
        {
            clock[0]++;

            if (clock[0] >= 500)
            {
                Board_Communi.Can_RC.ch0 = Board_Communi.Can_RC.ch1 = Board_Communi.Can_RC.ch2 = Board_Communi.Can_RC.ch3 = 0;
            }

            break;
        }

        default:
            break;
    }

    switch (Board_Communi.Rc[17])
    {
        case 1:
        {
            Board_Communi.Can_RC.sw = -((int16_t)(Board_Communi.Rc[9] << 8) | (Board_Communi.Rc[8]));
            Board_Communi.Can_RC.s1 = (int8_t)Board_Communi.Rc[10];
            Board_Communi.Can_RC.s2 = (int8_t)Board_Communi.Rc[11];
            Board_Communi.Can_RC.KV.key = (int16_t)(Board_Communi.Rc[13] << 8) | (Board_Communi.Rc[12]);
            Board_Communi.Rc[17] = 0;
            clock[1] = 0;
            break;
        }

        case 0:
        {
            clock[1]++;

            if (clock[1] >= 500)
            {
                Board_Communi.Can_RC.sw = 0;
                Board_Communi.Can_RC.KV.key = 0;
                Board_Communi.Can_RC.s1 = Board_Communi.Can_RC.s2 = 0;
            }

            break;
        }

        case 2:
        {
            Board_Communi.Can_RC.s1 = 3;
            Board_Communi.Can_RC.s2 = 1;
            Board_Communi.state.Grasp_Up = Board_Communi.Rc[0];     
            Board_Communi.state.Translation = Board_Communi.Rc[1];  
            Board_Communi.state.Telescoping = Board_Communi.Rc[2];  
            Board_Communi.state.Clap = Board_Communi.Rc[3];
            Board_Communi.state.Flip = Board_Communi.Rc[4];      
        }
    }
}

Board_Communi_t *Return_Board_Communi(void)
{
    return &Board_Communi;
}
