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
ElogErrCode elog_port_init(void) //!����漰������elogʹ����Դ�ĳ�ʼ�������綯̬������仺�����ڴ棬���Է��ڴ˽ӿ��У�Ŀǰ����Ĭ�ϡ�
{
    ElogErrCode result = ELOG_NO_ERR;

    /* add your code here */

    return result;
}

/**
 * output log port interface �����־�˿�
 *
 * @param log output of log ��־���
 * @param size log size ��־��С
 */
void elog_port_output(const char *log, size_t size)
{

    /* add your code here */
    //��־ʹ��printf�����printf�Ѿ��ض��򵽴���USART1
    //printf("%.*s", size, log);
    shellWriteEndLine(&shell, log, size); //!������Ϊ�����lettershell�������ʹlog���������Ӱ��shell��ָ��д��
}

/**
 * output lock
 */
void elog_port_output_lock(void)
{

    /* add your code here */
    //�ر�ȫ���ж�
    __set_PRIMASK(1);
}

/**
 * output unlock
 */
void elog_port_output_unlock(void)
{

    /* add your code here */
    //����ȫ���ж�
    __set_PRIMASK(0);
}

/**
 * get current time interface ��ȡ��ǰʱ��ӿ�
 *
 * @return current time ��ǰʱ��
 */
const char *elog_port_get_time(void)
{

    /* add your code here */
    return "";
}

/**
 * get current process name interface ��ȡ��ǰ�������ƽӿ�
 *
 * @return current process name ��ǰ��������
 */
const char *elog_port_get_p_info(void)
{

    /* add your code here */
    return "";
}

/**
 * get current thread name interface ��ȡ��ǰ�߳����ӿ�
 *
 * @return current thread name ��ǰ�̵߳�����
 */
const char *elog_port_get_t_info(void)
{

    /* add your code here */
    return "";
}

void Easylogge_Init(void)
{
    /* ��ʼ�� */
    elog_init();
    /* ʹ�ܲ�ɫ���� */
    elog_set_text_color_enabled(true);
    /* ����EasyLogger��־��ʽ */
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_T_INFO | ELOG_FMT_P_INFO));
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~(ELOG_FMT_FUNC | ELOG_FMT_T_INFO | ELOG_FMT_P_INFO));
    /* ����EasyLogger */
    elog_start();

    /* ���Բ�ɫ���� Hello EasyLogger! */
    log_a("����(Assert)"); //����(Assert)
    log_e("����(Error)"); //����(Error)
    log_w("����(Warn)"); //����(Warn)
    log_i("��Ϣ(Info)"); //��Ϣ(Info)
    log_d("����(Debug)"); //����(Debug)
    log_v("��ϸ(Verbose)"); //��ϸ(Verbose)
}
