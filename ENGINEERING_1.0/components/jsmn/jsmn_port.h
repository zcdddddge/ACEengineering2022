#ifndef __JSMN_PORT_H
#define __JSMN_PORT_H
#include "esp8266_send.h"

/*ÐÅÏ¢Êä³ö*/
#define JSMN_DEBUG_ON 1

#define JSMN_INFO(fmt, arg...) \
    do                         \
    {                          \
        if (JSMN_DEBUG_ON)     \
            log_i(fmt, ##arg); \
    } while (0)
#define JSMN_ERROR(fmt, arg...) \
    do                          \
    {                           \
        if (JSMN_DEBUG_ON)      \
            log_e(fmt, ##arg);  \
    } while (0)
#define JSMN_DEBUG(fmt, arg...) \
    do                          \
    {                           \
        if (JSMN_DEBUG_ON)      \
            log_d(fmt, ##arg);  \
    } while (0)

	
void jsmn_parser_init(void);
void json_weather_parse(uint8_t *json_str, weather_hourly_t *hourly);
#endif /* __JSMN_PORT_H */
