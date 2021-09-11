#include "jsmn_port.h"
#include "jsmn.h"
#include <stdio.h>
#include <string.h>
#include <elog.h>


jsmntok_t t[128];
static const char *json_string =
    "{\"user\": \"johndoe\", \"admin\": false, \"uid\": 1000,\n"
    "\"groups\": [\"users\", \"wheel\", \"audio\", \"video\"]}";
static const char *json_weather = "{\"results\":[{\"location\":{\"id\":\"WS0GHKN5ZP7T\",\"name\":\"Dongguan\",\"country\":\"CN\",\"path\":\"Dongguan,Dongguan,Guangdong,China\",\"timezone\":\"Asia/Shanghai\",\"timezone_offset\":\"+08:00\"},\"hourly\":[{\"time\":\"2021-02-26T12:00:00+08:00\",\"text\":\"Overcast\",\"code\":\"9\",\"temperature\":\"23\",\"humidity\":\"91\",\"wind_direction\":\"N\",\"wind_speed\":\"10\"}]}]}";                              



//void jsmn_parser_init(void)
//{
//	jsmn_init(&p);
//}

//数字字符串转换成整形
uint8_t str2int(char *str)
{
    switch (strlen((const char *)str))
    {
    case 1:
        return str[0] - 0x30;
    case 2:
        return (str[0] - 0x30) * 10 + (str[1] - 0x30);
    default:
        return 10;
    }
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0)
    {
        return 0;
    }
    return -1;
}

void jsmn_demo(void)
{
	int r;
	int i;
	jsmn_parser p; //jsmn解析器
    
	jsmn_init(&p);
	r = jsmn_parse(&p, json_string, strlen(json_string), t, sizeof(t) / sizeof(t[0]));

    if (r < 0)
    {
        log_e("Failed to parse JSON: %d\n", r); //解析JSON失败
        return;
    }
    /* 假设顶层元素是一个对象 */
    if (r < 1 || t[0].type != JSMN_OBJECT)
    {
        log_e("Object expected\n"); //缺少对象
        return;
    }
    /* 循环遍历根对象的所有键 */
    for (i = 1; i < r; i++)
    {
        if (jsoneq(json_string, &t[i], "user") == 0)
        {
            /* We may use strndup() to fetch string value */
            printf("- user: %.*s\n", t[i + 1].end - t[i + 1].start, 
                   json_string + t[i + 1].start);//在printf中使用,*表示用后面的形参替代*的位置，实现动态格式输出。
            i++;
        }
        else if (jsoneq(json_string, &t[i], "admin") == 0)
        {
            /* We may additionally check if the value is either "true" or "false" */
            printf("- Admin: %.*s\n", t[i + 1].end - t[i + 1].start,
                   json_string + t[i + 1].start);
            i++;
        }
        else if (jsoneq(json_string, &t[i], "uid") == 0)
        {
            /* We may want to do strtol() here to get numeric value */
            printf("- UID: %.*s\n", t[i + 1].end - t[i + 1].start,
                   json_string + t[i + 1].start);
            i++;
        }
        else if (jsoneq(json_string, &t[i], "groups") == 0)
        {
            int j;
            printf("- Groups:\n");
            if (t[i + 1].type != JSMN_ARRAY) //数组
            {
                continue;
            }
            for (j = 0; j < t[i + 1].size; j++)
            {
                jsmntok_t *g = &t[i + j + 2];
                printf("  * %.*s\n", g->end - g->start, json_string + g->start);
            }
            i += t[i + 1].size + 1;
        }
        else
        {
            printf("Unexpected key: %.*s\n", t[i].end - t[i].start,
                   json_string + t[i].start);
        }
    }
}


void json_weather_parse(uint8_t *json_str, weather_hourly_t *hourly)
{
	int r;
	int i;
	char buff[20];
	jsmn_parser p; //jsmn解析器
	
	memset(buff, '\0', sizeof(buff));
	jsmn_init(&p);
    r = jsmn_parse(&p, json_weather, strlen(json_weather), t, sizeof(t) / sizeof(t[0]));

    if (r < 0)
    {
        log_e("Failed to parse JSON: %d\n", r); //解析JSON失败
        return;
    }
    if (r < 1 || t[0].type != JSMN_OBJECT)
    {
        log_e("Object expected\n"); //缺少对象
        return;
    }
	
//	JSMN_INFO("[type][start][end][size]");
//	for(i = 0; i < r; i++)
//	{
//		JSMN_INFO("[%4d][%5d][%3d][%4d]", t[i].type, t[i].start, t[i].end, t[i].size);
//		JSMN_INFO(" - %.*s\n", t[i + 1].end - t[i + 1].start, json_weather + t[i + 1].start);
//	}
	
    /* 循环遍历根对象的所有键 */
    for (i = 1; i < r; i++)
    {
        if (jsoneq(json_weather, &t[i], "hourly") == 0)
        {
            i += 2;
			if(t[i].type == JSMN_OBJECT)
			{
				for (int j = i; j < r; j++)
				{
					if (jsoneq(json_weather, &t[j], "time") == 0)
					{
						strncpy(buff, json_weather + t[j + 1].start, t[j + 1].end - t[j + 1].start);
						JSMN_DEBUG("时间: - %s\n", buff);
						j++;
					}
					else if (jsoneq(json_weather, &t[j], "text") == 0)
					{
						strncpy(hourly->text, json_weather + t[j + 1].start, t[j + 1].end - t[j + 1].start);
						JSMN_DEBUG("天气现象文字: - %s\n", hourly->text);
						j++;
					}
					else if (jsoneq(json_weather, &t[j], "code") == 0)
					{
						strncpy(buff, json_weather + t[j + 1].start, t[j + 1].end - t[j + 1].start);
						hourly->code = str2int(buff);
						JSMN_DEBUG("天气现象代码: - %d\n", (int)hourly->code);
						memset(buff, '\0', sizeof(buff));
						j++;
					}
					else if (jsoneq(json_weather, &t[j], "temperature") == 0)
					{
						strncpy(buff, json_weather + t[j + 1].start, t[j + 1].end - t[j + 1].start);
						hourly->temperature = str2int(buff);
						JSMN_DEBUG("温度: - %d\n", (int)hourly->temperature);
						memset(buff, '\0', sizeof(buff));
						j++;
					}
					else if (jsoneq(json_weather, &t[j], "humidity") == 0)
					{
						strncpy(buff, json_weather + t[j + 1].start, t[j + 1].end - t[j + 1].start);
						hourly->humidity = str2int(buff);
						JSMN_DEBUG("相对湿度: - %d\n", (int)hourly->humidity);
						memset(buff, '\0', sizeof(buff));
						j++;
					}
					else if (jsoneq(json_weather, &t[j], "wind_direction") == 0)
					{
						strncpy(buff, json_weather + t[j + 1].start, t[j + 1].end - t[j + 1].start);
						JSMN_DEBUG("风向: - %s\n", buff);
						j++;
					}
					else if (jsoneq(json_weather, &t[j], "wind_speed") == 0)
					{
						strncpy(buff, json_weather + t[j + 1].start, t[j + 1].end - t[j + 1].start);
						hourly->wind_speed = str2int(buff);
						JSMN_DEBUG("风速: - %d\n", (int)hourly->wind_speed);
						memset(buff, '\0', sizeof(buff));
						j++;
					}
				}
				break;
			}
        }
    }
}


