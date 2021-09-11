#include "REMOTE_ISR.h"
#include "USART.h"


/*遥控数据结构体*/
RC_ctrl_t RC;


/*************************************************************************************************
*名称:	Get_Remote_Point
*功能:	返回遥控器控制变量，通过指针传递方式传递信息
*形参: 	无
*返回:	无
*说明:	用于处理遥控数据
*************************************************************************************************/
RC_ctrl_t *Return_Remote_Point(void)
{
    return &RC;
}



/*************************************************************************************************
*名称:	RC_Deal
*功能:	处理遥控数据
*形参: 	volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl
*返回:	无
*说明:	无
*************************************************************************************************/
static void RC_Deal(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	if(sbus_buf == 0 || rc_ctrl == 0)
	{
		return;
	}
	
	rc_ctrl->ch0 				= (dbus_rx_buffer[0] | (dbus_rx_buffer[1] << 8)) & 0x07ff; 						           							
	rc_ctrl->ch1			 	= ((dbus_rx_buffer[1] >> 3) | (dbus_rx_buffer[2] << 5)) & 0x07ff; 		                   				
	rc_ctrl->ch2 				= ((dbus_rx_buffer[2] >> 6) | (dbus_rx_buffer[3] << 2) | (dbus_rx_buffer[4] << 10)) & 0x07ff;	
	rc_ctrl->ch3 				= ((dbus_rx_buffer[4] >> 1) | (dbus_rx_buffer[5] << 7)) & 0x07ff;     					   							
	rc_ctrl->s1  				= ((dbus_rx_buffer[5] >> 4)& 0x000C) >> 2;													    											
	rc_ctrl->s2  				= ((dbus_rx_buffer[5] >> 4)& 0x0003); 														    													
  rc_ctrl->sw  				= (dbus_rx_buffer[16] | (dbus_rx_buffer[17] << 8))& 0x07ff;	
	rc_ctrl->KV.x		 		= dbus_rx_buffer[6] | (dbus_rx_buffer[7] << 8); 											    									
	rc_ctrl->KV.y 			= dbus_rx_buffer[8] | (dbus_rx_buffer[9] << 8);										       									
	rc_ctrl->KV.z 			= dbus_rx_buffer[10] | (dbus_rx_buffer[11] << 8); 										   										
	rc_ctrl->KV.press_l = dbus_rx_buffer[12]; 																    															
	rc_ctrl->KV.press_r = dbus_rx_buffer[13]; 																    														
	rc_ctrl->KV.key 		= dbus_rx_buffer[14] | (dbus_rx_buffer[15] << 8); 																					
	rc_ctrl->sw					= dbus_rx_buffer[16] | (dbus_rx_buffer[17] << 8);
	
	/*数值转换*/
	rc_ctrl->ch0 -= 1024;
	rc_ctrl->ch1 -= 1024;
	rc_ctrl->ch2 -= 1024;
	rc_ctrl->ch3 -= 1024;
	rc_ctrl->sw	 -= 1024;			
	/*标志位*/
	rc_ctrl->Flag = 1;
}



/*************************************************************************************************
*名称:	DMA2_Stream5_IRQHandler
*功能:	USART1中断
*形参: 	无
*返回:	无
*说明:	用于处理遥控数据
*************************************************************************************************/
void DMA2_Stream5_IRQHandler(void)		                                           
{
   if(DMA_GetFlagStatus(DMA2_Stream5,DMA_IT_TCIF5)==SET)                             
	 {
		DMA_ClearFlag(DMA2_Stream5,DMA_IT_TCIF5); 
		DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
		RC_Deal(dbus_rx_buffer,&RC);
	}
}



/*************************************************************************************************
*名称:	DMA2_Stream5_IRQHandler
*功能:	USART6中断
*形参: 	无
*返回:	无
*说明:	用于处理主控板1.0遥控数据
*************************************************************************************************/
void DMA2_Stream1_IRQHandler(void)
{
   if(DMA_GetFlagStatus(DMA2_Stream1,DMA_IT_TCIF1)==SET) 
	{
		DMA_ClearFlag(DMA2_Stream1,DMA_IT_TCIF1); 
		DMA_ClearITPendingBit(DMA2_Stream1,DMA_IT_TCIF1);
		RC_Deal(dbus_rx_buffer,&RC);
	}
}
