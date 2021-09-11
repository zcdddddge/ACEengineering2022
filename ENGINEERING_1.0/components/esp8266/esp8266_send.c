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

//uint8_t recv_data = 0;   //���ڴ��ڽ���
//uint8_t read_data = 0;   //���ڴ洢�ӻ�������ȡ��������
//lwrb_t usart3_ringbuff;  //���ڴ���3��ringbuff���
//#define USART3_BUFFDATA_SIZE  150 //����һ���ڴ����ڻ�����
//uint8_t usart3_buffdata[USART3_BUFFDATA_SIZE];


static void esp8266_connect_server(uint8_t *server);
static uint8_t esp8266_send_at(uint8_t *txData, uint8_t *rxData, uint16_t timeout);

//�����ַ���ת��������
#define STR_INT(str)  (uint8_t)((*(uint8_t *)str) - 0x30)



void esp8266_usart_init(void)
{
//	if(1 != lwrb_init(&usart3_ringbuff, (uint8_t*)usart3_buffdata, USART3_BUFFDATA_SIZE))
//	{
//		ESP_ERROR("usart1 ringbuff init fail.\r\n");
//	}
//	HAL_UART_Receive_IT(&huart1, (uint8_t*)&recv_data, 1);
	
	//jsmn_parser_init();
	
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                   //Ҫ�õ������жϷ�ʽ����Ҫ����IDLE�жϡ�
    HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //ʹ��DMA����
}

/**
  * @brief          ��ȡʱ��
  * @param[in]      none
  * @retval         none
  */
void esp8266_get_time(void)
{
	uint8_t *p, *p_start, *p_end;
	
    esp8266_connect_server((uint8_t *)ESP_CIPSTART_TIME);

//	esp8266_send_at((uint8_t *)GET_TIME, (uint8_t *)"OK", 200);   //����ʱ��
    while (1)
    {
        HAL_Delay(500);
        HAL_UART_Transmit(&huart3, (uint8_t *)"xxx", strlen((const char *)GET_TIME), 200);
		
		//Date: Thu, 25 Feb 2021 14:36:43 GMT
        if (UART3_ReFlag) //��ʱ�ٴνӵ�һ�����ݣ�Ϊʱ�������
        {
            if (strstr((char *)esp8266_buff, (char *)"Date") && strstr((char *)esp8266_buff, (char *)"GMT"))
            {
				p_start = (uint8_t *)strstr((char *)esp8266_buff, (char *)"Date");
                p_end = (uint8_t *)strstr((char *)esp8266_buff, (char *)"GMT");
                p = p_end - 9;
				
				ESP_INFO("�ɹ����յ�ʱ������");
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
            memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //����ջ���
            UART3_ReFlag = 0;
            HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //ʹ��DMA����
        }
    }
	memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //����ջ���
	UART3_ReFlag = 0;
	HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //ʹ��DMA����
	
}

/**
  * @brief          ��ȡСʱ����
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
	memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //����ջ���
	UART3_ReFlag = 0;
	HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //ʹ��DMA����
}

/**
  * @brief          ��ȡ����
  * @param[in]      none
  * @retval         none
  */
void esp8266_get_daily(void)
{
    esp8266_connect_server((uint8_t *)ESP_CIPSTART_WEATHER);

	HAL_UART_Transmit(&huart3, (uint8_t *)GET_SENIVERSE_DAILY, strlen((const char *)GET_TIME), 200);
}

/**
  * @brief          ͸��ģʽ
  * @param[in]      none
  * @retval         none
  */
void esp8266_passthrough_init(void)
{
    esp8266_connect_server((uint8_t *)ESP_CIPSTART);
}

static void esp8266_connect_server(uint8_t *server)
{
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                   //Ҫ�õ������жϷ�ʽ����Ҫ����IDLE�жϡ�
    HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //ʹ��DMA����

    esp8266_send_at((uint8_t *)"+++", (uint8_t *)"CLOSED", 200);  //������Ч
    esp8266_send_at((uint8_t *)ESP_RST, (uint8_t *)"ready", 400); //������Ч
    HAL_Delay(200);
    esp8266_send_at((uint8_t *)ESP_CWMODE1, (uint8_t *)"OK", 200);  //���� WIFI ģʽ (STA)
    esp8266_send_at((uint8_t *)ESP_RST, (uint8_t *)"ready", 400);   //������Ч
    esp8266_send_at((uint8_t *)ESP_CWJAP, (uint8_t *)"OK", 1000);   //���Ӷ���·���� - ��������� wifi  ESP_CIPMUX0
    esp8266_send_at((uint8_t *)ESP_CIPMUX0, (uint8_t *)"OK", 200);  //TCP ������
    esp8266_send_at(server, (uint8_t *)"OK", 200);                  //ģ�����ӵ� server
    esp8266_send_at((uint8_t *)ESP_CIPMODE1, (uint8_t *)"OK", 200); //����͸��ģʽ
    esp8266_send_at((uint8_t *)ESP_CIPSEND, (uint8_t *)"OK", 200);
}

static uint8_t esp8266_send_at(uint8_t *txData, uint8_t *rxData, uint16_t timeout)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)txData, strlen((const char *)txData), timeout);

    if (rxData && timeout) //��Ҫ�ȴ�Ӧ��
    {
        while (--timeout) //�ȴ�����ʱ
        {
            HAL_Delay(10);
            if (UART3_ReFlag) //���յ��ڴ���Ӧ����
            {
                esp8266_buff[esp_relen] = 0; //��ӽ�����
                if (strstr((const char *)esp8266_buff, (const char *)rxData))
                {
                    ESP_DEBUG("���ͳɹ������͵�ָ��Ϊ��%.*s", strlen((const char *)txData) - 2, (uint8_t *)txData);
                    ESP_DEBUG("���յ���ָ��Ϊ��%s\r\n", (uint8_t *)rxData);
					memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //����ջ���
					UART3_ReFlag = 0;
					HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //ʹ��DMA����
                    return 1;
                }
                memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //����ջ���
                UART3_ReFlag = 0;
                HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //ʹ��DMA����
            }
        }
    }
    memset(esp8266_buff, 0, ESP8266_BUFF_LEN); //����ջ���
    UART3_ReFlag = 0;
    HAL_UART_Receive_DMA(&huart3, esp8266_buff, ESP8266_BUFF_LEN); //ʹ��DMA����
    ESP_ERROR("����ʧ�ܣ����͵�ָ��Ϊ��%.*s", strlen((const char *)txData) - 2, (uint8_t *)txData);
    ESP_ERROR("���յ���ָ��Ϊ��%s\r\n", (uint8_t *)esp8266_buff);
    return 0;
}

/**
  * @brief          �����������USART3ȫ���ж�
  * @param[in]      none
  * @retval         none
  */
void usart3_irqhandler_dispose(void)
{
    uint32_t tmp_flag = 0;
    uint32_t temp;

    tmp_flag = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE); //��ȡIDLE��־λ

    /* ���ǲ���DMA�����жϷ�ʽ����Ӧ�¼���־ΪIDLE����⵽������·ʱ����λ��Ӳ���� 1�������ж� */
    if ((tmp_flag != RESET)) //idle��־����λ
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3); // �����־λ
        temp = huart3.Instance->SR;         // ���״̬�Ĵ���SR,��ȡSR�Ĵ�������ʵ�����SR�Ĵ����Ĺ���
        temp = huart3.Instance->DR;         // ��ȡ���ݼĴ����е�����

        HAL_UART_DMAStop(&huart3); // �ر�DMA,��ֹ�����ڼ�������

        temp = hdma_usart3_rx.Instance->NDTR; //! ��ȡDMA��δ��������ݸ�����NDTR�Ĵ��������ο����Ĳο��ֲ� ��DMA_Channel_TypeDef��  �����ͬ��оƬHAL�����涨��������е㲻ͬ
        esp_relen = ESP8266_BUFF_LEN - temp;  // �ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
        UART3_ReFlag = 1;                     // ������ɱ�־λ��1
    }
}
