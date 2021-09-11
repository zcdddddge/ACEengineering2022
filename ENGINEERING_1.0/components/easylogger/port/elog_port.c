/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */

#include <elog.h>
#include <stdio.h>
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "shell_port.h"
#include "shell.h"

extern Shell shell;

/**
 * EasyLogger port initialize 
 * 
 * @return result
 */
ElogErrCode elog_port_init(void) //!如果涉及到后续elog使用资源的初始化，比如动态申请分配缓冲区内存，可以放在此接口中，目前保持默认。
{
    ElogErrCode result = ELOG_NO_ERR;

    /* add your code here */

    return result;
}

/**
 * output log port interface 输出日志端口
 *
 * @param log output of log 日志输出
 * @param size log size 日志大小
 */
void elog_port_output(const char *log, size_t size)
{

    /* add your code here */
    //日志使用printf输出，printf已经重定向到串口USART1
    //printf("%.*s", size, log);
    shellWriteEndLine(&shell, log, size); //!这里是为了配合lettershell的输出，使log的输出不会影响shell的指令写入
}

/**
 * output lock
 */
void elog_port_output_lock(void)
{

    /* add your code here */
    //关闭全局中断
    __set_PRIMASK(1);
}

/**
 * output unlock
 */
void elog_port_output_unlock(void)
{

    /* add your code here */
    //开启全局中断
    __set_PRIMASK(0);
}

/**
 * get current time interface 获取当前时间接口
 *
 * @return current time 当前时间
 */
const char *elog_port_get_time(void)
{

    /* add your code here */
    return "";
}

/**
 * get current process name interface 获取当前进程名称接口
 *
 * @return current process name 当前进程名称
 */
const char *elog_port_get_p_info(void)
{

    /* add your code here */
    return "";
}

/**
 * get current thread name interface 获取当前线程名接口
 *
 * @return current thread name 当前线程的名字
 */
const char *elog_port_get_t_info(void)
{

    /* add your code here */
    return "";
}

void Easylogge_Init(void)
{
    /* 初始化 */
    elog_init();
    /* 使能彩色字体 */
    elog_set_text_color_enabled(true);
    /* 设置EasyLogger日志格式 */
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_T_INFO | ELOG_FMT_P_INFO));
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_T_INFO | ELOG_FMT_P_INFO));
    /* 开启EasyLogger */
    elog_start();

    /* 测试彩色字体 Hello EasyLogger! */
    log_a("断言(Assert)"); //断言(Assert)
    log_e("错误(Error)"); //错误(Error)
    log_w("警告(Warn)"); //警告(Warn)
    log_i("信息(Info)"); //信息(Info)
    log_d("调试(Debug)"); //调试(Debug)
    log_v("详细(Verbose)"); //详细(Verbose)
}
