#include "stm32f4xx.h"
#include "HardwareInit.h"
#include "TaskInit.h"
#include "CAN1_ISR.h"
#include "CAN2_ISR.h"
/************************** Dongguan-University of Technology -ACE**************************
 *@project   SentryChassis
 *@Author    Dongguan-University of Technology  ACE  ���䶫edddddge
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


/*��ʼ������*/
static TaskHandle_t Start_Handler;

/*��ʼ������*/
static void Start_Task(void *pvParameters)
{
		taskENTER_CRITICAL();                                   //�����ٽ���
		
		TaskInit();
	
		vTaskDelete(Start_Handler);                     			//ɾ����ʼ����
    taskEXIT_CRITICAL();                                 	//�˳��ٽ���
}

int main()
{
	HardwareInit();
//CAN1_201_To_204_SEND(500,500,500,500);
//CAN1_205_To_208_SEND(500,500,500,500);
//CAN2_205_To_208_SEND(500,500,500,500);
//CAN1_SEND_6020_7(500);
	xTaskCreate((TaskFunction_t )Start_Task,            	//������
			(const char*    )"Start_Task",                    //��������
			(uint16_t       )256,                  						//�����ջ��С
			(void*          )NULL,                            //���ݸ��������Ĳ���
			(UBaseType_t    )1,                 							//�������ȼ�
			(TaskHandle_t*  )&Start_Handler);            			//������  
			vTaskStartScheduler();                            //�����������
}


