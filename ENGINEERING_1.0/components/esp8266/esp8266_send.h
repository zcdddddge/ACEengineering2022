#ifndef  __BSP_ESP8266_H
#define	 __BSP_ESP8266_H
#include "main.h"

/*��Ϣ���*/
#define ESP_DEBUG_ON 1

#define ESP_INFO(fmt, arg...)  \
    do                         \
    {                          \
        if (ESP_DEBUG_ON)      \
            log_i(fmt, ##arg); \
    } while (0)
#define ESP_ERROR(fmt, arg...)  \
    do                          \
    {                           \
        if (ESP_DEBUG_ON)       \
            log_e(fmt, ##arg);  \
    } while (0)
#define ESP_DEBUG(fmt, arg...)  \
    do                          \
    {                           \
        if (ESP_DEBUG_ON)       \
            log_d(fmt, ##arg);  \
    } while (0)
	

#define ESP_AT          "AT\r\n"	
#define ESP_CWMODE1     "AT+CWMODE=1\r\n"		//STA+APģʽ
#define ESP_CWMODE2     "AT+CWMODE=2\r\n"		//STA+APģʽ
#define ESP_CWMODE3     "AT+CWMODE=3\r\n"		//STA+APģʽ
#define ESP_RST         "AT+RST\r\n"            //����
#define ESP_CIFSR       "AT+CIFSR\r\n"
#define ESP_CWJAP       "AT+CWJAP=\"caicai3\",\"cai81888223\"\r\n"  //����·����
//#define ESP_CIPSTART    "AT+CIPSTART=\"TCP\",\"183.230.40.39\",876\r\n"	//EDP������ 183.230.40.39/876
//#define ESP_CIPSTART    "AT+CIPSTART=\"TCP\",\"192.168.1.100\",8080\r\n"		//HTTP������183.230.40.33/80
//#define ESP_CIPSTART    "AT+CIPSTART=\"TCP\",\"192.168.3.6\",8080\r\n"
#define ESP_CIPSTART          "AT+CIPSTART=\"TCP\",\"192.168.3.37\",8080\r\n"  //192.168.3.37  192.168.4.1 
#define ESP_CIPSTART_TIME     "AT+CIPSTART=\"TCP\",\"www.beijing-time.org\",80\r\n"
#define ESP_CIPSTART_WEATHER  "AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80\r\n"
#define ESP_CIPMUX0     "AT+CIPMUX=0\r\n"
#define ESP_CIPMODE0    "AT+CIPMODE=0\r\n"		//��͸��ģʽ
#define ESP_CIPMODE1    "AT+CIPMODE=1\r\n"		//͸��ģʽ
#define ESP_CIPSEND     "AT+CIPSEND\r\n"        //��ʼ͸��
#define ESP_CIPSTATUS   "AT+CIPSTATUS\r\n"		//����״̬��ѯ

#define GET_SENIVERSE_DAILY  "GET https://api.seniverse.com/v3/weather/daily.json?key=SuhAluEGigCDwilHG&location=dongguan&language=en&unit=c&start=0&days=5\r\n"
#define GET_SENIVERSE_HOURLY "GET https://api.seniverse.com/v3/weather/hourly.json?key=SuhAluEGigCDwilHG&location=dongguan&language=en&unit=c&start=0&hours=1\r\n"
#define GET_TIME             "GET /time15.asp HTTP/1.1\r\nHost:www.beijing-time.org\n\n" //Date: Thu, 25 Feb 2021 13:02:10 GMT
//�˳�͸��ģʽ��
//+++
//AT+CIPMODE=0
//�Ͽ�TCP���ӣ�AT+CIPCLOSE
//�Ͽ�SSL���ӣ�AT+CIPCLOSE

//Date: Thu, 25 Feb 2021 14:36:43 GMT
typedef struct 
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t year;
	uint8_t month;
	uint8_t date;
	uint8_t week;		 
} network_time_t;


typedef struct
{
    uint8_t time;           //ʱ��
    char text[20];          //������������
    uint8_t code;           //�����������
    uint8_t temperature;    //�¶ȣ���λΪc���϶Ȼ�f���϶�
    uint8_t humidity;       //���ʪ�ȣ�0~100����λΪ�ٷֱ�
    uint8_t wind_direction; //����
    uint8_t wind_speed;     //���٣���λΪkm/h����ÿСʱ��mphӢ��ÿСʱ
} weather_hourly_t;

typedef struct
{
    uint8_t date;                  //����
    uint8_t text_day;              //����������������
    uint8_t code_day;              //���������������
    uint8_t text_night;            //���������������
    uint8_t code_night;            //��������������
    uint8_t high;                  //��������¶�
    uint8_t low;                   //��������¶�
    uint8_t rainfall;              //��ˮ������λmm
    uint8_t precip;                //��ˮ���ʣ���Χ0~100����λ�ٷֱȣ�Ŀǰ��֧�ֹ�����У�
    uint8_t wind_direction;        //��������
    uint8_t wind_direction_degree; //����Ƕȣ���Χ0~360
    uint8_t wind_speed;            //���٣���λkm/h����unit=cʱ����mph����unit=fʱ��
    uint8_t wind_scale;            //�����ȼ�
    uint8_t humidity;              //���ʪ�ȣ�0~100����λΪ�ٷֱ�
} weather_daily_t;


void esp8266_usart_init(void);
void esp8266_get_time(void);
void esp8266_get_hourly(void);
void esp8266_get_daily(void);
void esp8266_passthrough_init(void);
void usart3_irqhandler_dispose(void);

#endif


