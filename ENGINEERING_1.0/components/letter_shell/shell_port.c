/**
 * @brief     shell��ֲ��STM32L431ʱ�Ľӿ�ʵ��
 * @author    mculover666
 * @date      2021/02/07 
*/

/**
  * ������ʹ������ mobaxterm ���������Ϊ�����ն˽���ʱ��������Ҫע�����㣺
  *
  * 1.�����ն����������һ���ַ�ʱ������ַ��ᱻֱ��ͨ�����ڷ��ͣ���Ļ�ϲ�����ʾ�κζ�����
  * 2.�����Ƭ�����˻��ԣ��ն�������յ���Ż���ʾ������
  * 3.�����ն�����а��»س������˸����ɾ�������ĸ������ʱ����ֱ�ӷ������ֵ����Ƭ���п������ֵ�������Ƿ����˰�����
  *
  * ��lettershell������keys����ɲ鿴��ǰ֧�ֵİ�����ֵ����
  * ��lettershell�����롰tab���������Բ鿴��ǰ�����������
  */

#include "shell.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "shell_port.h"

/* 1. ����shell���󣬿���shell������ */
Shell shell;
char shell_buffer[512];
uint8_t recv_buf = 0;  //���ڽ��ջ�����


/* 2. �Լ�ʵ��shellд���� */

//shellд����ԭ�ͣ�typedef void (*shellWrite)(const char);
void User_Shell_Write(const char ch)
{
    //����STM32 HAL�� API ʹ�ò�ѯ��ʽ����
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
//	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 0xFFFF);
}

signed char User_Shell_Read(char *data)
{
	return 0;
}

/* 3. ��д��ʼ������ */
//TODO ��������ֲ���в���ϵͳ�Ĺ���Ҫ�޸�
void User_Shell_Init(void)
{
	//ʹ�ܴ����жϽ���
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&recv_buf, 1);
	
    //ע���Լ�ʵ�ֵ�д����
    shell.write = User_Shell_Write;

    //����shell��ʼ������
    shellInit(&shell, shell_buffer, 512);
}

/**
 * �жϻص�����
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* �ж����ĸ����ڴ������ж� */
    if (huart->Instance == USART1)
    {
        //����shell�������ݵĽӿ�
        shellHandler(&shell, recv_buf);
        //ʹ�ܴ����жϽ���
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&recv_buf, 1);
    }
	
//    /* �ж����ĸ����ڴ������ж� */
//    if(huart ->Instance == USART3)
//    {
//        /* �����յ�������д�뻺���� */
//        lwrb_write(&usart1_ringbuff, &recv_data, 1);
//        //����ʹ�ܴ��ڽ����ж�
//        HAL_UART_Receive_IT(huart, (uint8_t*)&recv_data, 1);
//    }
}






/**
  *   !!ע��:
  * 1.Ŀǰ֧�ֵĲ�������Ϊ���͡��ַ��͡��ַ����ͣ���֧�ָ����Ͳ�����
  * 2.������������������shell_cfg.h�еĺ궨�����ã�
  */
/**
  * @brief          letter-shell���Ժ���
  * @param[in]      i: ���һ������
  *                 ch: ���һ������
  *                 *str: ���һ������
  * @retval         none
  */
int letter_shell_demo(int i, char ch, char *str)
{
    printf("input int: %d, char: %c, string: %s\r\n", i, ch, str);

    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), letter_shell_demo, letter_shell_demo, letter_shell_demo);  //�����������б���

/* ��ӡϵͳ��Ϣ */
int sysinfo_print(void)
{
    uint32_t temp;

    printf("\r\n");

    temp = HAL_GetDEVID();             printf("HAL_GetDEVID            ------     0x%08x\r\n", temp);
    temp = HAL_GetHalVersion();        printf("HAL_GetHalVersion       ------     0x%08x\r\n", temp);
    temp = HAL_GetREVID();             printf("HAL_GetREVID            ------     0x%08x\r\n", temp);
    temp = HAL_GetUIDw0();             printf("HAL_GetUIDw0            ------     0x%08x\r\n", temp);
    temp = HAL_GetUIDw1();             printf("HAL_GetUIDw1            ------     0x%08x\r\n", temp);
    temp = HAL_GetUIDw2();             printf("HAL_GetUIDw2            ------     0x%08x\r\n", temp);
    temp = SystemCoreClock;            printf("SystemCoreClock         ------     0d%08d\r\n", temp);
    temp = HAL_RCC_GetHCLKFreq();      printf("HAL_RCC_GetHCLKFreq     ------     0d%08d\r\n", temp);
    temp = HAL_RCC_GetPCLK1Freq();     printf("HAL_RCC_GetPCLK1Freq    ------     0d%08d\r\n", temp);
    temp = HAL_RCC_GetPCLK2Freq();     printf("HAL_RCC_GetPCLK2Freq    ------     0d%08d\r\n", temp);
    temp = HAL_RCC_GetSysClockFreq();  printf("HAL_RCC_GetSysClockFreq ------     0d%08d\r\n", temp);

    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) | SHELL_CMD_DISABLE_RETURN, sysinfo_print, sysinfo_print, sysinfo_print);


