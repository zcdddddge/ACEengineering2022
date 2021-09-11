#include "TIM_ISR.h"

/*提供系统运行的时间*/
int16_t TIM3_I = 0,TIM3_sec = 0,TIM3_min = 0;
static int8_t led = -1;
void TIM3_IRQHandler(void)        
{
   if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) 
   {
			TIM3_I ++;
			if(TIM3_I >= 1000)
			{
				led = -led;
				TIM3_I = 0;
				TIM3_sec ++;
				if(TIM3_sec >= 60)
				{
					TIM3_min ++;
					TIM3_sec = 0;
				}
			}
			if(led == 1)
			{
				if(TIM3_I % 80 == 0)
				{
					GPIO_ToggleBits(GPIOE,GPIO_Pin_7);
				}
			}
			else if(led == -1)
			{
				GPIO_SetBits(GPIOE,GPIO_Pin_7);	
			}
   }
   TIM_ClearITPendingBit(TIM3,TIM_IT_Update); 
}
