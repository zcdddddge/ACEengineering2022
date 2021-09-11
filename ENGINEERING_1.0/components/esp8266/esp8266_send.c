#include "esp8266_send.h"
#include "usart.h"
#include <string.h>
#include "lwrb.h"
#include <elog.h>
#include "shell_port.h"
#include "lwrb.h"
#include "jsmn_port.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

#define ESP8266_BUFF_LEN 200
uint8_t esp8266_buff[ESP8266_BUFF_LEN] = "";
static uint8_t esp_relen;
static uint8_t UART3_ReFlag;

network_time_t net_time;
weather_daily_t weather_daily;
weather_hourly_t weather_hourly;

//uint8_t recv_data = 0;   //用于串口接收
//uint8_t read_data = 0;   //用于存储从缓冲区读取出的数据
//lwrb_t usart3_ringbuff;  //用于串口3的ringbuff句柄
//#define USART3_BUFFDATA_SIZE  150 //开辟一块内存用于缓冲区
//uint8_t usart3_buffdata[USART3_BUFFDATA_SIZE];


static void esp8266_connect_server(uint8_t *server);
static uint8_t esp8266_send_at(uint8_t *txData, uint8_t *rxData, uint16_t timeout);

//数字字符串转换成整形
#define STR_INT(str)  (uint8_t)((*(uint8_t *)str) - 0x30)



void esp8266_usart_init(void)
{
//	if(1 != lwrb_init(&usart3_ringbuff, (uint8_t*)usart3_buffdata, USART3_BUFFDATA_SIZE))
//	{
//		ESP_ERROR("usart1 ringbuff init fail.\r\n");
//	}
//	HAL_UART_Receive_IT(&huart1, (uint8_t*)&recv_data, 1);
	
	//jsmn_parser_init();
	
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                   //要用到空闲中断方式，即要开启IDLE中断。
    HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //使能DMA接收
}

/**
  * @brief          获取时间
  * @param[in]      none
  * @retval         none
  */
void esp8266_get_time(void)
{
	uint8_t *p, *p_start, *p_end;
	
    esp8266_connect_server((uint8_t *)ESP_CIPSTART_TIME);

//	esp8266_send_at((uint8_t *)GET_TIME, (uint8_t *)"OK", 200);   //返回时间
    while (1)
    {
        HAL_Delay(500);
        HAL_UART_Transmit(&huart3, (uint8_t *)"xxx", strlen((const char *)GET_TIME), 200);
		
		//Date: Thu, 25 Feb 2021 14:36:43 GMT
        if (UART3_ReFlag) //此时再次接到一次数据，为时间的数据
        {
            if (strstr((char *)esp8266_buff, (char *)"Date") && strstr((char *)esp8266_buff, (char *)"GMT"))
            {
				p_start = (uint8_t *)strstr((char *)esp8266_buff, (char *)"Date");
                p_end = (uint8_t *)strstr((char *)esp8266_buff, (char *)"GMT");
                p = p_end - 9;
				
				ESP_INFO("成功接收到时间数据");
                ESP_INFO("%.*s\r\n", p_end-p_start, p_start);
                
				net_time.hour = STR_INT(p) * 10 + STR_INT(p + 1)/* + 8*/; //GMT0-->GMT8
                net_time.min = STR_INT(p + 3) * 10 + STR_INT(p + 4);
                net_time.sec = STR_INT(p + 6) * 10 + STR_INT(p + 7);
                net_time.year = STR_INT(p - 5) * 1000 + STR_INT(p - 4) * 100 + STR_INT(p - 3) * 10 + STR_INT(p - 2);
                net_time.date = STR_INT(p - 12) * 10 + STR_INT(p - 11);
                
				if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Jan"))
                    net_time.month = 1;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Feb"))
                    net_time.month = 2;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Mar"))
                    net_time.month = 3;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Apr"))
                    net_time.month = 4;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"May"))
                    net_time.month = 5;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Jun"))
                    net_time.month = 6;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Jul"))
                    net_time.month = 7;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Aug"))
                    net_time.month = 8;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Sep"))
                    net_time.month = 9;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Oct"))
                    net_time.month = 10;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Nov"))
                    net_time.month = 11;
                else if ((uint8_t *)strstr((char *)esp8266_buff, (char *)"Dec"))
                    net_time.month = 12;
				
				break;
            }
            memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //清接收缓存
            UART3_ReFlag = 0;
            HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //使能DMA接收
        }
    }
	memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //清接收缓存
	UART3_ReFlag = 0;
	HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //使能DMA接收
	
}

/**
  * @brief          获取小时天气
  * @param[in]      none
  * @retval         none
  */
void esp8266_get_hourly(void)
{
    esp8266_connect_server((uint8_t *)ESP_CIPSTART_WEATHER);

	HAL_UART_Transmit(&huart3, (uint8_t *)GET_SENIVERSE_HOURLY, strlen((const char *)GET_SENIVERSE_HOURLY), 200);
	
    while (!UART3_ReFlag)
	{
		HAL_Delay(500);
		HAL_UART_Transmit(&huart3, (uint8_t *)GET_SENIVERSE_HOURLY, strlen((const char *)GET_SENIVERSE_HOURLY), 200);
	}
	json_weather_parse(esp8266_buff, &weather_hourly);
	memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //清接收缓存
	UART3_ReFlag = 0;
	HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //使能DMA接收
}

/**
  * @brief          获取天气
  * @param[in]      none
  * @retval         none
  */
void esp8266_get_daily(void)
{
    esp8266_connect_server((uint8_t *)ESP_CIPSTART_WEATHER);

	HAL_UART_Transmit(&huart3, (uint8_t *)GET_SENIVERSE_DAILY, strlen((const char *)GET_TIME), 200);
}

/**
  * @brief          透传模式
  * @param[in]      none
  * @retval         none
  */
void esp8266_passthrough_init(void)
{
    esp8266_connect_server((uint8_t *)ESP_CIPSTART);
}

static void esp8266_connect_server(uint8_t *server)
{
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                   //要用到空闲中断方式，即要开启IDLE中断。
    HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //使能DMA接收

    esp8266_send_at((uint8_t *)"+++", (uint8_t *)"CLOSED", 200);  //重启生效
    esp8266_send_at((uint8_t *)ESP_RST, (uint8_t *)"ready", 400); //重启生效
    HAL_Delay(200);
    esp8266_send_at((uint8_t *)ESP_CWMODE1, (uint8_t *)"OK", 200);  //设置 WIFI 模式 (STA)
    esp8266_send_at((uint8_t *)ESP_RST, (uint8_t *)"ready", 400);   //重启生效
    esp8266_send_at((uint8_t *)ESP_CWJAP, (uint8_t *)"OK", 1000);   //连接对象路由器 - 连入局域网 wifi  ESP_CIPMUX0
    esp8266_send_at((uint8_t *)ESP_CIPMUX0, (uint8_t *)"OK", 200);  //TCP 单连接
    esp8266_send_at(server, (uint8_t *)"OK", 200);                  //模块连接到 server
    esp8266_send_at((uint8_t *)ESP_CIPMODE1, (uint8_t *)"OK", 200); //开启透传模式
    esp8266_send_at((uint8_t *)ESP_CIPSEND, (uint8_t *)"OK", 200);
}

static uint8_t esp8266_send_at(uint8_t *txData, uint8_t *rxData, uint16_t timeout)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)txData, strlen((const char *)txData), timeout);

    if (rxData && timeout) //需要等待应答
    {
        while (--timeout) //等待倒计时
        {
            HAL_Delay(10);
            if (UART3_ReFlag) //接收到期待的应答结果
            {
                esp8266_buff[esp_relen] = 0; //添加结束符
                if (strstr((const char *)esp8266_buff, (const char *)rxData))
                {
                    ESP_DEBUG("发送成功，发送的指令为：%.*s", strlen((const char *)txData) - 2, (uint8_t *)txData);
                    ESP_DEBUG("接收到的指令为：%s\r\n", (uint8_t *)rxData);
					memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //清接收缓存
					UART3_ReFlag = 0;
					HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //使能DMA接收
                    return 1;
                }
                memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //清接收缓存
                UART3_ReFlag = 0;
                HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //使能DMA接收
            }
        }
    }
    memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //清接收缓存
    UART3_ReFlag = 0;
    HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //使能DMA接收
    ESP_ERROR("发送失败，发送的指令为：%.*s", strlen((const char *)txData) - 2, (uint8_t *)txData);
    ESP_ERROR("接收到的指令为：%s\r\n", (uint8_t *)esp8266_buff);
    return 0;
}

/**
  * @brief          这个函数处理USART3全局中断
  * @param[in]      none
  * @retval         none
  */
void usart3_irqhandler_dispose(void)
{
    uint32_t tmp_flag = 0;
    uint32_t temp;

    tmp_flag = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE); //获取IDLE标志位

    /* 就是采用DMA空闲中断方式，对应事件标志为IDLE，检测到空闲线路时，该位由硬件置 1，生成中断 */
    if ((tmp_flag != RESET)) //idle标志被置位
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3); // 清除标志位
        temp = huart3.Instance->SR;         // 清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
        temp = huart3.Instance->DR;         // 读取数据寄存器中的数据

        HAL_UART_DMAStop(&huart3); // 关闭DMA,防止处理期间有数据

        temp = hdma_usart3_rx.Instance->NDTR; //! 获取DMA中未传输的数据个数，NDTR寄存器分析参考中文参考手册 （DMA_Channel_TypeDef）  这个不同的芯片HAL库里面定义的命名有点不同
        esp_relen = ESP8266_BUFF_LEN - temp;  // 总计数减去未传输的数据个数，得到已经接收的数据个数
        UART3_ReFlag = 1;                     // 接受完成标志位置1
    }
}
