/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       fifo_buff.c/h
  * @brief      ��������
  * @note       
  * @history    2021.07.08
  *
  @verbatim   
  ==============================================================================
  ���� ����+DMA+IDLE�ж�+�������У���ߴ��ڽ���Ч��  (�ο�����ӥ̸��Ƭ�����ںš�)
  1�����ڳ�ʼ������һ��ִ����ɣ����ھͿ�ʼʹ�� DMA�������ݣ������жϲ���ʱ��
    �û������ں����õ� DMA������յ����ݡ�
  2����Ϊ DMA���ݵĸ����ɴ��ڿ����жϾ���������һ��һ֡���ݺܳ�
    ��������Ϊ���� 128������һ֡���ݴ���ʣ�໺��ռ䣩����ô����ᷢ���������
    �����ñ�־λ����Щ��������޷����֣�����������Ļ����С���ñȽϹؼ���
    �������һ֡���棬�����֡���ϣ������� 2 ���ݴη���
  3������ڴ������ƣ��޷������󻺴棬��ô���Կ��� DMA�İ봫���жϣ�
    ����Ҳ���Լ�ʱȡ�� DMA��������ݣ�����ʹ�ö�ʱ���µķ�ʽ����
  4���û����� buff_read �����������ã�û�����ƣ���Ϊ�˽�ʡ�ڴ棬
    һ��С�ڵ��� DMA �Ľ��ջ��� usart_buff_rx�������ڸ������У�
    buff_read ��û��������ݣ����԰��������
  5��fifo_read_buff ����ֵΪʵ�ʽ��յ������ݳ��ȣ�������� 0������û�н��յ��κ����ݣ�
    ���Ҷ�ȡ��֮�󣬻��Զ���� DMA��������ݣ�
    �û�����Ҫ�������ʵ���ϣ���������ݻ��ڣ�ֻ���û���ȡ���ˣ������ջᱻ������յ����������ǣ�
  6�������ж�һ���������Ϊ������ȼ�����Ϊ�� DMA��̨�Զ����յģ�
    �����ж����ȼ���Ͳ����ᶪʧ���ݣ�ǰ���ǻ����㹻�󣩡�
  7�����ʹ�ô��ڲ�Ϊ�գ�USART_IT_RXNE���жϣ�һ����ջ���� ORE����
    ��ʱ���������ô���ᵼ���������󣬵�һ�� DMA�����ܼ�ʱ�������ݣ�
    Ӧ�ò�������ô��󣬵�Ϊ�˷������������Ҳ�����˴����־��
  �ܽ�һ�£����ڽ��յĹؼ����� DMAѭ�����գ��ͽ��������ĸ��¡�
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  */
#include "fifo_buff.h"
#include <limits.h>

#define IS_POWER_OF_2(x) ((x) != 0 && (((x) & ((x)-1)) == 0))

// ������ʹ�ú꣬����ȷ��û��ʹ������
// �����������������������Ӧ��min_fifo�ĳ�������ǳ���Ϥ�������������������Ƚ����ף���Ҳ��effective c�Ȿ�����Ƽ���������
static inline uint32_t min_fifo(uint32_t X, uint32_t Y)
{
    return ((X) > (Y) ? (Y) : (X));
}

/**
  * @brief          �������������뵽��һ��2���ݵ������
  * @param[in]      Num: Ҫ���ĵ�����
  * @retval         ���ظ��ĺ������
  * @attention
  */
uint32_t roundup_pow_of_two(uint32_t Num)
{
    uint32_t result = 1;

    if (IS_POWER_OF_2(Num) || Num == 0)
        return Num;
    else if (Num > LONG_MAX)
        return (LONG_MAX ^ ULONG_MAX); // WARN: ���Num����(LONG_MAX+1)����ôresult������(LONG_MAX+1)

    while (Num)
    {
        Num >>= 1;
        result <<= 1;
    }

    return result;
}

/**
  * @brief          �������������뵽��һ��2���ݵ���С��
  * @param[in]      Num: Ҫ���ĵ�����
  * @retval         ���ظ��ĺ������
  * @attention
  */
uint32_t rounddown_pow_of_two(uint32_t Num)
{
    uint32_t result = 1;

    if (IS_POWER_OF_2(Num) || Num == 0)
        return Num;
    else if (Num > LONG_MAX)
        return (LONG_MAX ^ ULONG_MAX); // WARN: ���Num����(LONG_MAX+1)����ôresult������(LONG_MAX+1)

    while (Num)
    {
        Num >>= 1;
        result <<= 1;
    }

    return result >> 1;
}

/**
  * @brief          ���λ������ĳ�ʼ��
  * @param[in]      pfifo: ��ѭ����������ʼ��
  * @param[in]      buff: ���ڴ洢���ݵĻ��λ�����
  * @param[in]      size: �������Ĵ�С
  * @retval         �ɹ�����0
  * @attention
  */
int32_t fifo_init(fifo_rx_def *pfifo, uint8_t *buff, uint32_t size)
{
    assert_param(pfifo != NULL || buff != NULL);

    if (!IS_POWER_OF_2(size)) // ���� 2 ���ݴη�
    {
        return -1;
    }

    pfifo->in = 0;
    pfifo->out = 0;
    pfifo->buffer = buff;
    pfifo->size = size; // ����������ô�С
    pfifo->last_cnt = size;

    return 0;
}

/**
  * @brief          �ӻ��λ�������ȡ����
  * @param[in]      pfifo: �洢���ݵĻ��λ�����
  * @param[in]      buffer: ���洢����ѭ�������������ݵ�Ŀ�껺����
  * @param[in]      len: Ҫ��ѭ���������л�ȡ�ĳ���
  * @retval         ��ѭ����������õ�ʵ�ʳ���
  * @attention
  */
uint32_t fifo_read_buff(fifo_rx_def *pfifo, uint8_t *buffer, uint32_t len)
{
    uint32_t length;

    assert_param(pfifo != NULL || pfifo->buffer != NULL || buffer != NULL);

    len = min_fifo(len, pfifo->in - pfifo->out); //��ȡ����out��ʼ����������Ĵ�С

    /* first get the data from pfifo->out until the end of the buffer */
    length = min_fifo(len, pfifo->size - (pfifo->out & (pfifo->size - 1))); //��ȡ����out��ʼ����������Ĵ�С
    memcpy(buffer, pfifo->buffer + (pfifo->out & (pfifo->size - 1)), length);

    /* then get the rest (if any) from the beginning of the buffer */
    memcpy(buffer + length, pfifo->buffer, len - length);

    pfifo->out += len;

    return len;
}

/**
  * @brief          �����ݷ��뻷�λ�����
  * @param[in]      pfifo: �洢���ݵĻ��λ�����
  * @param[in]      buffer: Ҫ�洢�����λ������е�����
  * @param[in]      len: Ҫ�洢�����λ������е����ݳ���
  * @retval         �洢�ڻ��λ������е�ʵ�ʴ�С
  * @attention
  */
uint32_t fifo_write_buff(fifo_rx_def *pfifo, uint8_t *buffer, uint32_t len)
{
    uint32_t length;

    len = min_fifo(len, (pfifo->size - (pfifo->in - pfifo->out)));

    length = min_fifo(len, pfifo->size - (pfifo->in & (pfifo->size - 1)));

    memcpy(pfifo->buffer + (pfifo->in & pfifo->size - 1), buffer, length);
    memcpy(pfifo->buffer, buffer + length, len - length);

    pfifo->in += len;

    return len;
}

/**
  * @brief          ��ȡ���λ������Ŀ����ڴ��С
  * @param[in]      pfifo: �洢���ݵĻ��λ�����
  * @retval         ���λ������Ŀ����ڴ��С
  * @attention
  */
unsigned int fifo_get_free(fifo_rx_def *pfifo)
{
    return ((pfifo->size > 0) ? (pfifo->size - (pfifo->in - pfifo->out)) : 0);
}

/**
  * @brief          ��ȡ���λ�������ʹ�õ��ڴ��С
  * @param[in]      pfifo: �洢���ݵĻ��λ�����
  * @retval         ���λ�������ʹ�õ��ڴ��С
  * @attention
  */
unsigned int fifo_get_full(fifo_rx_def *pfifo)
{
    return (pfifo->in - pfifo->out);
}

/**
  * @brief          ��黷�λ������Ƿ�Ϊ��
  * @param[in]      pfifo: �洢���ݵĻ��λ�����
  * @retval         ���û�����ݾͷ���1
  * @attention
  */
unsigned int fifo_is_empty(fifo_rx_def *pfifo)
{
    return ((pfifo->size > 0) && (pfifo->in == pfifo->out));
}

/**
  * @brief          ��黷�λ������Ƿ�����
  * @param[in]      pfifo: �洢���ݵĻ��λ�����
  * @retval         ������˾ͷ���1
  * @attention
  */
unsigned int fifo_is_full(fifo_rx_def *pfifo)
{
    return ((pfifo->size == 0) || (pfifo->size == (pfifo->in - pfifo->out)));
}
