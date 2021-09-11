#ifndef  __BSP_ESP8266_H
#define	 __BSP_ESP8266_H
#include "main.h"

/*信息输出*/
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
#define ESP_CWMODE1     "AT+CWMODE=1\r\n"		//STA+AP模式
#define ESP_CWMODE2     "AT+CWMODE=2\r\n"		//STA+AP模式
#define ESP_CWMODE3     "AT+CWMODE=3\r\n"		//STA+AP模式
#define ESP_RST         "AT+RST\r\n"            //重启
#define ESP_CIFSR       "AT+CIFSR\r\n"
#define ESP_CWJAP       "AT+CWJAP=\"caicai3\",\"cai81888223\"\r\n"  //连接路由器
//#define ESP_CIPSTART    "AT+CIPSTART=\"TCP\",\"183.230.40.39\",876\r\n"	//EDP服务器 183.230.40.39/876
//#define ESP_CIPSTART    "AT+CIPSTART=\"TCP\",\"192.168.1.100\",8080\r\n"		//HTTP服务器183.230.40.33/80
//#define ESP_CIPSTART    "AT+CIPSTART=\"TCP\",\"192.168.3.6\",8080\r\n"
#define ESP_CIPSTART          "AT+CIPSTART=\"TCP\",\"192.168.3.37\",8080\r\n"  //192.168.3.37  192.168.4.1 
#define ESP_CIPSTART_TIME     "AT+CIPSTART=\"TCP\",\"www.beijing-time.org\",80\r\n"
#define ESP_CIPSTART_WEATHER  "AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80\r\n"
#define ESP_CIPMUX0     "AT+CIPMUX=0\r\n"
#define ESP_CIPMODE0    "AT+CIPMODE=0\r\n"		//非透传模式
#define ESP_CIPMODE1    "AT+CIPMODE=1\r\n"		//透传模式
#define ESP_CIPSEND     "AT+CIPSEND\r\n"        //开始透传
#define ESP_CIPSTATUS   "AT+CIPSTATUS\r\n"		//网络状态查询

#define GET_SENIVERSE_DAILY  "GET https://api.seniverse.com/v3/weather/daily.json?key=SuhAluEGigCDwilHG&location=dongguan&language=en&unit=c&start=0&days=5\r\n"
#define GET_SENIVERSE_HOURLY "GET https://api.seniverse.com/v3/weather/hourly.json?key=SuhAluEGigCDwilHG&location=dongguan&language=en&unit=c&start=0&hours=1\r\n"
#define GET_TIME             "GET /time15.asp HTTP/1.1\r\nHost:www.beijing-time.org\n\n" //Date: Thu, 25 Feb 2021 13:02:10 GMT
//退出透传模式：
//+++
//AT+CIPMODE=0
//断开TCP连接：AT+CIPCLOSE
//断开SSL连接：AT+CIPCLOSE

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
    uint8_t time;           //时间
    char text[20];          //天气现象文字
    uint8_t code;           //天气现象代码
    uint8_t temperature;    //温度，单位为c摄氏度或f华氏度
    uint8_t humidity;       //相对湿度，0~100，单位为百分比
    uint8_t wind_direction; //风向
    uint8_t wind_speed;     //风速，单位为km/h公里每小时或mph英里每小时
} weather_hourly_t;

typedef struct
{
    uint8_t date;                  //日期
    uint8_t text_day;              //白天天气现象文字
    uint8_t code_day;              //白天天气现象代码
    uint8_t text_night;            //晚间天气现象文字
    uint8_t code_night;            //晚间天气现象代码
    uint8_t high;                  //当天最高温度
    uint8_t low;                   //当天最低温度
    uint8_t rainfall;              //降水量，单位mm
    uint8_t precip;                //降水概率，范围0~100，单位百分比（目前仅支持国外城市）
    uint8_t wind_direction;        //风向文字
    uint8_t wind_direction_degree; //风向角度，范围0~360
    uint8_t wind_speed;            //风速，单位km/h（当unit=c时）、mph（当unit=f时）
    uint8_t wind_scale;            //风力等级
    uint8_t humidity;              //相对湿度，0~100，单位为百分比
} weather_daily_t;


void esp8266_usart_init(void);
void esp8266_get_time(void);
void esp8266_get_hourly(void);
void esp8266_get_daily(void);
void esp8266_passthrough_init(void);
void usart3_irqhandler_dispose(void);

#endif


