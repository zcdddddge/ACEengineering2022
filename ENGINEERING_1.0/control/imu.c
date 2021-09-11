#include "imu.h"
#include "usart1.h"
#include "fifo_buff.h"
#include "parameter.h"

imu_data_t imu_data;
extern fifo_rx_def fifo_usart_rx_1;
fifo_rx_def *pfifo_imu = &fifo_usart_rx_1;

const imu_data_t *get_imu_control_point(void)
{
    return &imu_data;
}

void imu_control_init(void)
{
    usart1_init(2, 2);
}

/*
*功能：发送数据到陀螺仪模块进行初始化校准（云台串口一）
*协议：帧头  校验段0x12 0x34  帧尾
*/
void imu_zero_correct(void)
{
    uint8_t SendBuff_Correct[2];
    SendBuff_Correct[0] = 0x12;
    SendBuff_Correct[1] = 0x34;
	
	usart1_dma_send(SendBuff_Correct, 2);
}

/*
*功能：给陀螺仪喂狗粮（云台串口一）
*协议：0x01
*/

void gyro_usart_iwdg(void)
{
    uint8_t gyro[1];
    gyro[0] = 0x01;
	
	usart1_dma_send(gyro, 1);
}

/* 接收陀螺仪数值 */
//int temp_1, temp_2, Re_time; //TODO 测试
void imu_data_reception(void)
{
	uint8_t imu_buff_read[32];

    //temp_2 = xTaskGetTickCount();	//获取当前系统时间
    //Re_time = temp_2 - temp_1;
	
	uint32_t length_imu = fifo_read_buff(pfifo_imu, imu_buff_read, 32);
    if (length_imu)
    {
		imu_data.length = length_imu;
		for (uint16_t i = 0; i < length_imu; i++)
		{
			if (imu_buff_read[i] == 0xfe && imu_buff_read[i + 13] == 0xee)
			{
				imu_data.Gyro_X = ((short)(imu_buff_read[i + 1] << 8 | imu_buff_read[i + 2])) / 100.0f;
				imu_data.Gyro_Y = ((short)(imu_buff_read[i + 3] << 8 | imu_buff_read[i + 4])) / 100.0f;
				imu_data.Gyro_Z = -((short)(imu_buff_read[i + 5] << 8 | imu_buff_read[i + 6])) / 100.0f; //陀螺仪模块朝上，加负号，反之
				imu_data.roll_angle = ((short)(imu_buff_read[i + 7] << 8 | imu_buff_read[i + 8])) / 100.0f;
				imu_data.pitch_angle = ((short)(imu_buff_read[i + 9] << 8 | imu_buff_read[i + 10])) / 100.0f;
				imu_data.yaw_angle = -((short)(imu_buff_read[i + 11] << 8 | imu_buff_read[i + 12])) / 100.0f; //陀螺仪模块朝上，加负号，反之

				i = i + 13;
			}
		}		
	}
	else
	{
		//没有数据
	}
	
	if (pfifo_imu->error)
    {
		// 接收错误
        pfifo_imu->error = 0; 
    }
 
    //temp_1 = xTaskGetTickCount();	//获取当前系统时间
}

volatile unsigned char PYR_rx_buffer[64];

PYR_t PYR;

/*返回底盘陀螺仪数据指针*/
PYR_t* Return_PYR_t(void)
{
	return &PYR;
}


/*底盘陀螺仪数据处理函数*/
static void PYR_Deal(u8 num)
{
	u8 i;
	u16 n;
	u8 sum;
	for(i=0; i<num; i++)
	{
		if((PYR.data[i] == 0x55) && (PYR.data[i+11] == 0x55) && (PYR.data[i+22] == 0x55))	//校验3组数据头
		{
			if(PYR.data[i+23] == 0x53)
			{
				n = i+22;		//重置坐标
				sum = 0x55 + 0x53 + PYR.data[n+2] + PYR.data[n+3] + PYR.data[n+4] + 
				PYR.data[n+5] + PYR.data[n+6] + PYR.data[n+7] + PYR.data[n+8] 
				+ PYR.data[n+9];
				if(PYR.data[n+10] != sum)	//检查校验和
				return;
					
				//PYR.Pitch = ((PYR.data[n+3]<< 8) | PYR.data[n+2])/32768.0*180;
				//PYR.Roll = ((PYR.data[n+5]<< 8) | PYR.data[n+4])/32768.0*180; 
				PYR.Yaw = ((PYR.data[n+7]<< 8) | PYR.data[n+6])/32768.0*180;  // 角速度计算|偏航角(z轴) 
				PYR.Yaw_Var = PYR.Yaw - PYR.Yaw_Lock;
				if(PYR.Yaw_Var > 300)
				{
					PYR.Yaw_Var = -(360 - PYR.Yaw + PYR.Yaw_Lock);
				}
				else if(PYR.Yaw_Var <  -300)
				{
					PYR.Yaw_Var = 360 + PYR.Yaw - PYR.Yaw_Lock;
				}
				PYR.flag = 1;
			}
		}
	}
	
}
	

/*uart4中断函数*/
void UART4_IRQHandler(void)
{
	u16 i;
	u8 num=0;
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
	{
		DMA_Cmd(DMA1_Stream2, DISABLE);
		while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
		num	=	UART4->SR;
		num	=	UART4->DR;
		num	=	64-DMA_GetCurrDataCounter(DMA1_Stream5);
		for(i=0;i<num;i++)
		{
			PYR.data[i] = PYR_rx_buffer[i];
		}
		PYR.flag = 2;
		PYR_Deal(num);
		DMA_SetCurrDataCounter(DMA1_Stream2,64);
		DMA_Cmd(DMA1_Stream2, ENABLE);
	}
}



