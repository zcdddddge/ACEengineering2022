/* - (SFUD��ֲ�ӿ�
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (c) 2018, zylx, <qgyhd1234@gmail.com>
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
 * Created on: 2018-11-23
 */

#include <sfud.h>
#include <stdarg.h>
#include <stdio.h>
#include <stm32F4xx_hal.h>
#include <stm32F4xx_hal_gpio.h>
#include <string.h>
#include "spi.h"
#include "shell_port.h"

void sfud_log_info(const char *format, ...);
//sfud_err qspi_send_then_recv(const void *send_buf, size_t send_length, void *recv_buf, size_t recv_length);
extern SPI_HandleTypeDef hspi1;
;

typedef struct
{
    SPI_HandleTypeDef *spix;
    GPIO_TypeDef *cs_gpiox;
    uint16_t cs_gpio_pin;
} spi_user_data, *spi_user_data_t;

static char log_buf[256];

void sfud_log_debug(const char *file, const long line, const char *format, ...);

static void spi_lock(const sfud_spi *spi)
{
    __disable_irq();
}

static void spi_unlock(const sfud_spi *spi)
{
    __enable_irq();
}

/**
 * SPI ��д
 */
static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf, size_t read_size)
{
    sfud_err result = SFUD_SUCCESS;
    uint8_t send_data, read_data;

    spi_user_data_t spi_dev = (spi_user_data_t)spi->user_data;

    if (write_size)
    {
        SFUD_ASSERT(write_buf);
    }
    if (read_size)
    {
        SFUD_ASSERT(read_buf);
    }

    /* reset cs pin */
    if (spi_dev->cs_gpiox != NULL)
        HAL_GPIO_WritePin(spi_dev->cs_gpiox, spi_dev->cs_gpio_pin, GPIO_PIN_RESET);

    do
    {
        /* ��ʼ��д���� */
        for (size_t i = 0, retry_times; i < write_size + read_size; i++)
        {
            /* ��д�������е����ݵ� SPI ���ߣ�����д�����д dummy(0xFF) �� SPI ���� */
            if (i < write_size)
            {
                send_data = *write_buf++;
            }
            else
            {
                send_data = SFUD_DUMMY_DATA;
            }

            /* �������� */
            retry_times = 1000;
            while (__HAL_SPI_GET_FLAG(spi_dev->spix, SPI_FLAG_TXE) == RESET)
            {
                SFUD_RETRY_PROCESS(NULL, retry_times, result);
            }
            if (result != SFUD_SUCCESS)
            {
                break;
            }
            //SPI_I2S_SendData(spi_dev->spix, send_data); �⺯��
            //HAL_SPI_Transmit(spi_dev->spix, &send_data, read_size, retry_times);
            WRITE_REG(spi_dev->spix->Instance->DR, send_data);

            /* �������� */
            retry_times = 1000;
            while (__HAL_SPI_GET_FLAG(spi_dev->spix, SPI_FLAG_RXNE) == RESET)
            {
                SFUD_RETRY_PROCESS(NULL, retry_times, result);
            }
            if (result != SFUD_SUCCESS)
            {
                break;
            }
            //read_data = SPI_I2S_ReceiveData(spi_dev->spix); �⺯��
            read_data = READ_REG(spi_dev->spix->Instance->DR);

            /* д�������е����ݷ�����ٶ�ȡ SPI �����е����ݵ��������� */
            if (i >= write_size)
            {
                *read_buf++ = read_data;
            }
        }
    } while (0);

    /* set cs pin */
    if (spi_dev->cs_gpiox != NULL)
        HAL_GPIO_WritePin(spi_dev->cs_gpiox, spi_dev->cs_gpio_pin, GPIO_PIN_SET);

    return result;
}

#ifdef SFUD_USING_QSPI //ʹ��QSPIͨѶģʽ
/**
 * ͨ��QSPI��ȡflash����
 */
static sfud_err qspi_read(const struct __sfud_spi *spi, uint32_t addr, sfud_qspi_read_cmd_format *qspi_read_cmd_format, uint8_t *read_buf, size_t read_size)
{

    sfud_err result = SFUD_SUCCESS;
    QSPI_CommandTypeDef Cmdhandler;
    extern QSPI_HandleTypeDef hqspi;

    /* set cmd struct */
    Cmdhandler.Instruction = qspi_read_cmd_format->instruction;
    if (qspi_read_cmd_format->instruction_lines == 0)
    {
        Cmdhandler.InstructionMode = QSPI_INSTRUCTION_NONE;
    }
    else if (qspi_read_cmd_format->instruction_lines == 1)
    {
        Cmdhandler.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    }
    else if (qspi_read_cmd_format->instruction_lines == 2)
    {
        Cmdhandler.InstructionMode = QSPI_INSTRUCTION_2_LINES;
    }
    else if (qspi_read_cmd_format->instruction_lines == 4)
    {
        Cmdhandler.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    }

    Cmdhandler.Address = addr;
    Cmdhandler.AddressSize = QSPI_ADDRESS_24_BITS;
    if (qspi_read_cmd_format->address_lines == 0)
    {
        Cmdhandler.AddressMode = QSPI_ADDRESS_NONE;
    }
    else if (qspi_read_cmd_format->address_lines == 1)
    {
        Cmdhandler.AddressMode = QSPI_ADDRESS_1_LINE;
    }
    else if (qspi_read_cmd_format->address_lines == 2)
    {
        Cmdhandler.AddressMode = QSPI_ADDRESS_2_LINES;
    }
    else if (qspi_read_cmd_format->address_lines == 4)
    {
        Cmdhandler.AddressMode = QSPI_ADDRESS_4_LINES;
    }

    Cmdhandler.AlternateBytes = 0;
    Cmdhandler.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    Cmdhandler.AlternateBytesSize = 0;

    Cmdhandler.DummyCycles = qspi_read_cmd_format->dummy_cycles;

    Cmdhandler.NbData = read_size;
    if (qspi_read_cmd_format->data_lines == 0)
    {
        Cmdhandler.DataMode = QSPI_DATA_NONE;
    }
    else if (qspi_read_cmd_format->data_lines == 1)
    {
        Cmdhandler.DataMode = QSPI_DATA_1_LINE;
    }
    else if (qspi_read_cmd_format->data_lines == 2)
    {
        Cmdhandler.DataMode = QSPI_DATA_2_LINES;
    }
    else if (qspi_read_cmd_format->data_lines == 4)
    {
        Cmdhandler.DataMode = QSPI_DATA_4_LINES;
    }

    Cmdhandler.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    Cmdhandler.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    Cmdhandler.DdrMode = QSPI_DDR_MODE_DISABLE;
    Cmdhandler.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    HAL_QSPI_Command(&hqspi, &Cmdhandler, 5000);

    if (HAL_QSPI_Receive(&hqspi, read_buf, 5000) != HAL_OK)
    {
        sfud_log_info("qspi recv data failed(%d)!", hqspi.ErrorCode);
        hqspi.State = HAL_QSPI_STATE_READY;
        result = SFUD_ERR_READ;
    }

    return result;
}
#endif /* SFUD_USING_QSPI */

/* about 100 microsecond delay */
static void retry_delay_100us(void)
{
    uint32_t delay = 120;
    while (delay--)
        ;
}

static spi_user_data spi1 = {.spix = &hspi1, .cs_gpiox = GPIOG, .cs_gpio_pin = GPIO_PIN_6};
sfud_err sfud_spi_port_init(sfud_flash *flash)
{
    sfud_err result = SFUD_SUCCESS;

    /**
     * add your port spi bus and device object initialize code like this:
     * 1. rcc initialize
     * 2. gpio initialize
     * 3. spi device initialize
     * 4. flash->spi and flash->retry item initialize
     *    flash->spi.wr = spi_write_read; //Required
     *    flash->spi.qspi_read = qspi_read; //Required when QSPI mode enable
     *    flash->spi.lock = spi_lock;
     *    flash->spi.unlock = spi_unlock;
     *    flash->spi.user_data = &spix;
     *    flash->retry.delay = null;
     *    flash->retry.times = 10000; //Required
     */
    switch (flash->index)
    {
    //��sfud_cfg.h�е����
    case SFUD_W25Q128_DEVICE_INDEX:
    {
        /* 1.RCC��2.GPIO��3.SPI�����ʼ��HAL���Ѿ�ʵ���� */
        /* 4.���ýӿں����� */
        flash->spi.wr = spi_write_read;
        //flash->spi.qspi_read = qspi_read;
        flash->spi.lock = spi_lock;
        flash->spi.unlock = spi_unlock;
        flash->spi.user_data = &spi1;
        /* ��Լ100΢���ӳ� */
        flash->retry.delay = retry_delay_100us;
        /* ��Լ60�볬ʱ */
        flash->retry.times = 60 * 10000;

        break;
    }
    }

    return result;
}

/**
 * ��������Ǵ�ӡ������Ϣ��
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 */
void sfud_log_debug(const char *file, const long line, const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD](%s:%ld) ", file, line);
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\n", log_buf);
    shellWriteEndLine(&shell, ("\r\n"), sizeof("\r\n"));
    va_end(args);
}

/**
 * ��������Ǵ�ӡ������Ϣ��
 *
 * @param format output format
 * @param ... args
 */
void sfud_log_info(const char *format, ...)
{
    extern Shell shell;
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD]");
    //shellWriteEndLine(&shell, ("[SFUD]"), sizeof("[SFUD]"));
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\n", log_buf);
    //shellWriteEndLine(&shell, log_buf, sizeof(log_buf));
    shellWriteEndLine(&shell, ("\r\n"), sizeof("\r\n"));
    va_end(args);
}

/**
 * SFUD��ʾ�ĵ�һ�������豸���ԡ�
 *
 * @param  addr: flash����ʼ��ַ
 * @param  size: ����flash��С
 * @param  *data: ����flash���ݻ�����
 */
#define SFUD_DEMO_TEST_BUFFER_SIZE 1024
//static uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE];  //sfud���Ի�����
//sfud_demo(0, sizeof(sfud_demo_test_buf), sfud_demo_test_buf);
int sfud_demo(uint32_t addr) //���� - 0
{
    uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE]; //sfud���Ի�����
    size_t size = sizeof(sfud_demo_test_buf);
    uint8_t *data = sfud_demo_test_buf;

    sfud_err result = SFUD_SUCCESS;
    /* sfud_get_device_table()
	 * ��ȡ Flash �豸������ SFUD �����ļ��лᶨ�� Flash �豸��
	 * ���������н�Ҫʹ�õ� Flash �豸����
	 * ���� SFUD ֧�ֶ�� Flash �豸ͬʱ������
	 * ������ͨ�� Flash �豸λ���豸��������ֵ������ Flash �豸����
	 * �����豸��Χ���� NULL ��
	 */
    const sfud_flash *flash = sfud_get_device_table() + 0;
    size_t i;

    /* ׼��д���� */
    for (i = 0; i < size; i++)
    {
        data[i] = i;
    }

    /* ɾ������ */
    result = sfud_erase(flash, addr, size);
    if (result == SFUD_SUCCESS)
    {
        //printf("Erase the %s flash data finish. Start from 0x%08X, size is %d.\r\n", flash->name, addr, size);
        printf("����%s����������ɡ� ��0x%08X��ʼ����СΪ%d��\r\n", flash->name, addr, size);
    }
    else
    {
        //printf("Erase the %s flash data failed.\r\n", flash->name);
        printf("����%s��������ʧ�ܡ�\r\n", flash->name);
        return 0;
    }

    /* д���� */
    result = sfud_write(flash, addr, size, data);
    if (result == SFUD_SUCCESS)
    {
        //printf("Write the %s flash data finish. Start from 0x%08X, size is %d.\r\n", flash->name, addr, size);
        printf("д��%s flash������ɡ���0x%08X��ʼ����СΪ%d��\r\n", flash->name, addr, size);
    }
    else
    {
        //printf("Write the %s flash data failed.\r\n", flash->name);
        printf("д��%s flash����ʧ�ܡ�\r\n", flash->name);
        return 0;
    }

    /* ������ */
    result = sfud_read(flash, addr, size, data);
    if (result == SFUD_SUCCESS)
    {
        //printf("Read the %s flash data success. Start from 0x%08X, size is %d. The data is:\r\n", flash->name, addr, size);
        //printf("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
        printf("��ȡ%s flash data�ɹ�����0x%08X��ʼ����СΪ%d����������:\r\n", flash->name, addr, size);
        printf("ƫ����(h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
        for (i = 0; i < size; i++)
        {
            if (i % 16 == 0)
            {
                printf("[%08X] ", addr + i);
            }
            printf("%02X ", data[i]);
            if (((i + 1) % 16 == 0) || i == size - 1)
            {
                printf("\r\n");
            }
        }
        printf("\r\n");
    }
    else
    {
        //printf("Read the %s flash data failed.\r\n", flash->name);
        printf("��ȡ%s��������ʧ�ܡ�\r\n", flash->name);
    }

    /* ���ݼ�� */
    for (i = 0; i < size; i++)
    {
        if (data[i] != i % 256)
        {
            //printf("Read and check write data has an error. Write the %s flash data failed.\r\n", flash->name);
            printf("��ȡ�ͼ��д�����ݴ���д��%s flash����ʧ�ܡ�\r\n", flash->name);
            break;
        }
    }
    if (i == size)
    {
        //printf("The %s flash test is success.\r\n", flash->name);
        printf("%s flash���Գɹ���\r\n", flash->name);
    }

    return 0;
}
//!��������shell
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), sfud_demo, sfud_demo, sfud_demo); //�����������б���
