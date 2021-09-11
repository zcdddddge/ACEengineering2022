/**
 * @brief     shell移植到STM32L431时的接口实现
 * @author    mculover666
 * @date      2021/02/07 
*/

/**
  * 当我们使用诸如 mobaxterm 这类软件作为串口终端交互时，首先需要注意三点：
  *
  * 1.当在终端软件中输入一个字符时，这个字符会被直接通过串口发送，屏幕上不会显示任何东西；
  * 2.如果单片机做了回显，终端软件中收到后才会显示出来；
  * 3.当在终端软件中按下回车键、退格键、删除键、四个方向键时，会直接发送其键值，单片机中靠这个键值来解析是否按下了按键。
  *
  * 在lettershell中输入keys命令即可查看当前支持的按键键值解析
  * 在lettershell中输入“tab”键，可以查看当前的命令（函数）
  */

#include "shell.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "shell_port.h"

/* 1. 创建shell对象，开辟shell缓冲区 */
Shell shell;
char shell_buffer[512];
uint8_t recv_buf = 0;  //串口接收缓冲区


/* 2. 自己实现shell写函数 */

//shell写函数原型：typedef void (*shellWrite)(const char);
void User_Shell_Write(const char ch)
{
    //调用STM32 HAL库 API 使用查询方式发送
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
//	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 0xFFFF);
}

signed char User_Shell_Read(char *data)
{
	return 0;
}

/* 3. 编写初始化函数 */
//TODO 这里在移植到有操作系统的工程要修改
void User_Shell_Init(void)
{
	//使能串口中断接收
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&recv_buf, 1);
	
    //注册自己实现的写函数
    shell.write = User_Shell_Write;

    //调用shell初始化函数
    shellInit(&shell, shell_buffer, 512);
}

/**
 * 中断回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 判断是哪个串口触发的中断 */
    if (huart->Instance == USART1)
    {
        //调用shell处理数据的接口
        shellHandler(&shell, recv_buf);
        //使能串口中断接收
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&recv_buf, 1);
    }
	
//    /* 判断是哪个串口触发的中断 */
//    if(huart ->Instance == USART3)
//    {
//        /* 将接收到的数据写入缓冲区 */
//        lwrb_write(&usart1_ringbuff, &recv_data, 1);
//        //重新使能串口接收中断
//        HAL_UART_Receive_IT(huart, (uint8_t*)&recv_data, 1);
//    }
}






/**
  *   !!注意:
  * 1.目前支持的参数类型为整型、字符型、字符串型，不支持浮点型参数。
  * 2.函数最大传入参数个数由shell_cfg.h中的宏定义配置：
  */
/**
  * @brief          letter-shell测试函数
  * @param[in]      i: 随便一个东西
  *                 ch: 随便一个东西
  *                 *str: 随便一个东西
  * @retval         none
  */
int letter_shell_demo(int i, char ch, char *str)
{
    printf("input int: %d, char: %c, string: %s\r\n", i, ch, str);

    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), letter_shell_demo, letter_shell_demo, letter_shell_demo);  //导出到命令列表里

/* 打印系统信息 */
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


