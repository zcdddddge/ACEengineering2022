#include "iwdg.h"
#include "led.h"

/*
 * 设置 IWDG 的超时时间
 * Tout = prv/40 * rlv (s)
 *      prv可以是[4,8,16,32,64,128,256]
 * prv:预分频器值，取值如下：
 *     @arg IWDG_Prescaler_4: IWDG prescaler set to 4
 *     @arg IWDG_Prescaler_8: IWDG prescaler set to 8
 *     @arg IWDG_Prescaler_16: IWDG prescaler set to 16
 *     @arg IWDG_Prescaler_32: IWDG prescaler set to 32
 *     @arg IWDG_Prescaler_64: IWDG prescaler set to 64
 *     @arg IWDG_Prescaler_128: IWDG prescaler set to 128
 *     @arg IWDG_Prescaler_256: IWDG prescaler set to 256
 *
 * rlv:预分频器值，取值范围为：0-0XFFF
 * 函数调用举例：
 * IWDG_Init(IWDG_Prescaler_64 ,625);  // IWDG 1s 超时溢出
 */

void iwdg_init(u8 prer, u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对寄存器IWDG_PR和IWDG_RLR的写操作

    IWDG_SetPrescaler(prer); //设置IWDG预分频值: 可对计数器时钟频率进行分频，分频系数最大为256

    IWDG_SetReload(rlr); //设置IWDG重装载值: 当计数器计数到终值 (0x000) 时会产生一个复位信号，计数器寄存器将装载重新计数。

    IWDG_ReloadCounter(); //按照IWDG重装载寄存器的值重装载IWDG计数器

    IWDG_Enable(); //使能IWDG
}

//喂独立看门狗
void iwdg_feed(void)
{
    // 把重装载寄存器的值放到计数器中，喂狗，防止IWDG复位
    // 当计数器的值减到0的时候会产生系统复位
    IWDG_ReloadCounter();
}
