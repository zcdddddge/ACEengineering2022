#include "stm32f4xx.h"
#include "HardwareInit.h"
#include "TaskInit.h"
#include "CAN1_ISR.h"
#include "CAN2_ISR.h"
/************************** Dongguan-University of Technology -ACE**************************
 *@project   SentryChassis
 *@Author    Dongguan-University of Technology  ACE  刘沅东edddddge
 *@Date      2021-04-17

                    P                          :u7  :Ii              .
                   QBQ                     sQBBQBB  PBBBBQI.        XQBBBBBBBBBQBBBBBBBBBBBBM
                  bBBBZ                 .MQBQBBBQB  5BBBBBBBBi      uBBBBBBBBBQBBBBBBBBBQBBBP
                 bBBQQB5               XBBBRQQBBBP  sQBQBQQBBBZ     IBBBBBBBBBBBBBBBBBBBBBBBD
                 rBBgRQBY             BBQQRBBQr        rgBBBQr
               .  iBBgRQB7           BBQRgBQ:            iE.
              :BY  7BBgRQB:         sBQMgBB
             .BBB:  uBBgRBB.        BBMDQQ:                         rSU57  UQPdPbPPPPqPPbPdQs
             BBQBB:  XBQgRBB        QBggQB                          sBEQ1  QBBBBQBBBBBBBBBBBZ
            BBQgBBB   KBRDRBB       BBgDBB                          jBDQU  QBBBBBBBBBBBQBBBBg
           BBQgRBB     dQggQBB      BBggQB.                         iXJS7  uDK5XXK5KXKXXSSXg7
          gBQgRQB   BBggQDggQBQ     YBQDMBB
         PBQgRBB   BBBBBRQgMgQBg     BBQgRBB:            iZ:
        2BQgMBB.  BBBBBBBBBQRgQBK     BBBRQBBQL.      .rRBBQBr       ..                   ..
       vQBgRQB:  :uriiiiiirBQQgBB1     XQBQQQBBBBE  uBQBQBQBBBD     SBBBBBBBBBBBBBBBBBBBQBBBD
      7QBQBBBr             :BBBQBBY     .ZBQBBBBBB  qBBQBBBBB:      UBBBBQBBBBBBBBBBBBBBBBBBd
     LBBBBBBJ               7BBBBBQu       YRBBBQB  KBBBBBJ.        IBQBBBBBQBBBBBBBBBBBBBBBZ
                                                7i  .7.
*************************** Dongguan-University of Technology -ACE**************************/


/*开始任务句柄*/
static TaskHandle_t Start_Handler;

/*开始任务函数*/
static void Start_Task(void *pvParameters)
{
		taskENTER_CRITICAL();                                   //进入临界区
		
		TaskInit();
	
		vTaskDelete(Start_Handler);                     			//删除开始任务
    taskEXIT_CRITICAL();                                 	//退出临界区
}

int main()
{
	HardwareInit();
//CAN1_201_To_204_SEND(500,500,500,500);
//CAN1_205_To_208_SEND(500,500,500,500);
//CAN2_205_To_208_SEND(500,500,500,500);
//CAN1_SEND_6020_7(500);
	xTaskCreate((TaskFunction_t )Start_Task,            	//任务函数
			(const char*    )"Start_Task",                    //任务名称
			(uint16_t       )256,                  						//任务堆栈大小
			(void*          )NULL,                            //传递给任务函数的参数
			(UBaseType_t    )1,                 							//任务优先级
			(TaskHandle_t*  )&Start_Handler);            			//任务句柄  
			vTaskStartScheduler();                            //开启任务调度
}


