/************************************ ��ݸ��ѧԺACEʵ���� ************************************
 *@project   ����������
 *@Author    ��ݸ��ѧԺ  ACE������ս��  ���Դ
 *@Date      2021-07-18
                                                                                                    
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
************************************ ��ݸ��ѧԺACEʵ���� ************************************/

#include "main.h"
#include "init.h"
#include "chassis_app.h"
#include "gimbal_app.h"
#include "struct_typedef.h"
#include "parameter.h"
#include "led.h"
#include "pwm.h"
#include "iwdg.h"
#include "init.h"

int main(void)
{
//	if (get_sys_cfg() == GIMBAL_APP)
//	{
//		friction_wheel_init(); //Ħ���ֳ�ʼ��
//	}
    start_task();          //ϵͳ��ʼ��
    vTaskStartScheduler(); //�����������

    while (1)
    {
        ;
    }
}
