#include "iwdg.h"
#include "led.h"

/*
 * ���� IWDG �ĳ�ʱʱ��
 * Tout = prv/40 * rlv (s)
 *      prv������[4,8,16,32,64,128,256]
 * prv:Ԥ��Ƶ��ֵ��ȡֵ���£�
 *     @arg IWDG_Prescaler_4: IWDG prescaler set to 4
 *     @arg IWDG_Prescaler_8: IWDG prescaler set to 8
 *     @arg IWDG_Prescaler_16: IWDG prescaler set to 16
 *     @arg IWDG_Prescaler_32: IWDG prescaler set to 32
 *     @arg IWDG_Prescaler_64: IWDG prescaler set to 64
 *     @arg IWDG_Prescaler_128: IWDG prescaler set to 128
 *     @arg IWDG_Prescaler_256: IWDG prescaler set to 256
 *
 * rlv:Ԥ��Ƶ��ֵ��ȡֵ��ΧΪ��0-0XFFF
 * �������þ�����
 * IWDG_Init(IWDG_Prescaler_64 ,625);  // IWDG 1s ��ʱ���
 */

void iwdg_init(u8 prer, u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����

    IWDG_SetPrescaler(prer); //����IWDGԤ��Ƶֵ: �ɶԼ�����ʱ��Ƶ�ʽ��з�Ƶ����Ƶϵ�����Ϊ256

    IWDG_SetReload(rlr); //����IWDG��װ��ֵ: ����������������ֵ (0x000) ʱ�����һ����λ�źţ��������Ĵ�����װ�����¼�����

    IWDG_ReloadCounter(); //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������

    IWDG_Enable(); //ʹ��IWDG
}

//ι�������Ź�
void iwdg_feed(void)
{
    // ����װ�ؼĴ�����ֵ�ŵ��������У�ι������ֹIWDG��λ
    // ����������ֵ����0��ʱ������ϵͳ��λ
    IWDG_ReloadCounter();
}
