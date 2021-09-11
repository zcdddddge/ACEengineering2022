/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       fifo_buff.c/h
  * @brief      无锁队列
  * @note       
  * @history    2021.07.08
  *
  @verbatim   
  ==============================================================================
  利用 串口+DMA+IDLE中断+无锁队列，提高串口接收效率  (参考《鱼鹰谈单片机公众号》)
  1、串口初始化函数一旦执行完成，串口就开始使用 DMA接收数据，空闲中断产生时，
    用户才能在后续得到 DMA缓存接收的数据。
  2、因为 DMA数据的更新由串口空闲中断决定，所以一旦一帧数据很长
    （在这里为大于 128，或者一帧数据大于剩余缓存空间），那么程序会发现这个错误，
    并设置标志位（有些错误可能无法发现），所以这里的缓存大小设置比较关键，
    必须大于一帧缓存，最好两帧以上，并且是 2 的幂次方。
  3、如果内存有限制，无法开更大缓存，那么可以开启 DMA的半传输中断，
    这样也可以及时取走 DMA缓存的数据（或者使用定时更新的方式）。
  4、用户缓存 buff_read 可以随意设置，没有限制，但为了节省内存，
    一般小于等于 DMA 的接收缓存 usart_buff_rx。另外在该例子中，
    buff_read 并没有清除数据，可以按需清除。
  5、fifo_read_buff 返回值为实际接收到的数据长度，如果等于 0，代表没有接收到任何数据，
    并且读取完之后，会自动清除 DMA缓存的数据，
    用户不需要清除它（实际上，缓存的数据还在，只是用户读取不了，并最终会被后面接收的数据所覆盖）
  6、串口中断一般可以设置为最低优先级，因为是 DMA后台自动接收的，
    所以中断优先级最低并不会丢失数据（前提是缓存足够大）。
  7、如果使用串口不为空（USART_IT_RXNE）中断，一般接收会出现 ORE错误，
    此时如果不清除该错误会导致死机现象，但一般 DMA总是能及时接收数据，
    应该不会产生该错误，但为了发现这种情况，也设置了错误标志。
  总结一下，串口接收的关键就是 DMA循环接收，和接收索引的更新。
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
  */
#include "fifo_buff.h"
#include <limits.h>

#define IS_POWER_OF_2(x) ((x) != 0 && (((x) & ((x)-1)) == 0))

// 不建议使用宏，除非确定没有使用隐患
// 建议用内联函数，除非你对应用min_fifo的场景陷阱非常熟悉，否则还是用内联函数比较稳妥，这也是effective c这本书所推荐的做法。
static inline uint32_t min_fifo(uint32_t X, uint32_t Y)
{
    return ((X) > (Y) ? (Y) : (X));
}

/**
  * @brief          计算能四舍五入到下一个2次幂的最大数
  * @param[in]      Num: 要更改的数字
  * @retval         返回更改后的数字
  * @attention
  */
uint32_t roundup_pow_of_two(uint32_t Num)
{
    uint32_t result = 1;

    if (IS_POWER_OF_2(Num) || Num == 0)
        return Num;
    else if (Num > LONG_MAX)
        return (LONG_MAX ^ ULONG_MAX); // WARN: 如果Num大于(LONG_MAX+1)，那么result将等于(LONG_MAX+1)

    while (Num)
    {
        Num >>= 1;
        result <<= 1;
    }

    return result;
}

/**
  * @brief          计算能四舍五入到下一个2次幂的最小数
  * @param[in]      Num: 要更改的数字
  * @retval         返回更改后的数字
  * @attention
  */
uint32_t rounddown_pow_of_two(uint32_t Num)
{
    uint32_t result = 1;

    if (IS_POWER_OF_2(Num) || Num == 0)
        return Num;
    else if (Num > LONG_MAX)
        return (LONG_MAX ^ ULONG_MAX); // WARN: 如果Num大于(LONG_MAX+1)，那么result将等于(LONG_MAX+1)

    while (Num)
    {
        Num >>= 1;
        result <<= 1;
    }

    return result >> 1;
}

/**
  * @brief          环形缓冲区的初始化
  * @param[in]      pfifo: 将循环缓冲区初始化
  * @param[in]      buff: 用于存储数据的环形缓冲区
  * @param[in]      size: 缓冲区的大小
  * @retval         成功返回0
  * @attention
  */
int32_t fifo_init(fifo_rx_def *pfifo, uint8_t *buff, uint32_t size)
{
    assert_param(pfifo != NULL || buff != NULL);

    if (!IS_POWER_OF_2(size)) // 必须 2 的幂次方
    {
        return -1;
    }

    pfifo->in = 0;
    pfifo->out = 0;
    pfifo->buffer = buff;
    pfifo->size = size; // 必须最后设置大小
    pfifo->last_cnt = size;

    return 0;
}

/**
  * @brief          从环形缓冲区获取数据
  * @param[in]      pfifo: 存储数据的环形缓冲区
  * @param[in]      buffer: 将存储来自循环缓冲区的数据的目标缓冲区
  * @param[in]      len: 要从循环缓冲区中获取的长度
  * @retval         从循环缓冲区获得的实际长度
  * @attention
  */
uint32_t fifo_read_buff(fifo_rx_def *pfifo, uint8_t *buffer, uint32_t len)
{
    uint32_t length;

    assert_param(pfifo != NULL || pfifo->buffer != NULL || buffer != NULL);

    len = min_fifo(len, pfifo->in - pfifo->out); //获取队列out开始到数组结束的大小

    /* first get the data from pfifo->out until the end of the buffer */
    length = min_fifo(len, pfifo->size - (pfifo->out & (pfifo->size - 1))); //获取队列out开始到数组结束的大小
    memcpy(buffer, pfifo->buffer + (pfifo->out & (pfifo->size - 1)), length);

    /* then get the rest (if any) from the beginning of the buffer */
    memcpy(buffer + length, pfifo->buffer, len - length);

    pfifo->out += len;

    return len;
}

/**
  * @brief          将数据放入环形缓冲区
  * @param[in]      pfifo: 存储数据的环形缓冲区
  * @param[in]      buffer: 要存储到环形缓冲区中的数据
  * @param[in]      len: 要存储到环形缓冲区中的数据长度
  * @retval         存储在环形缓冲区中的实际大小
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
  * @brief          获取环形缓冲区的可用内存大小
  * @param[in]      pfifo: 存储数据的环形缓冲区
  * @retval         环形缓冲区的可用内存大小
  * @attention
  */
unsigned int fifo_get_free(fifo_rx_def *pfifo)
{
    return ((pfifo->size > 0) ? (pfifo->size - (pfifo->in - pfifo->out)) : 0);
}

/**
  * @brief          获取环形缓冲区已使用的内存大小
  * @param[in]      pfifo: 存储数据的环形缓冲区
  * @retval         环形缓冲区已使用的内存大小
  * @attention
  */
unsigned int fifo_get_full(fifo_rx_def *pfifo)
{
    return (pfifo->in - pfifo->out);
}

/**
  * @brief          检查环形缓冲区是否为空
  * @param[in]      pfifo: 存储数据的环形缓冲区
  * @retval         如果没有数据就返回1
  * @attention
  */
unsigned int fifo_is_empty(fifo_rx_def *pfifo)
{
    return ((pfifo->size > 0) && (pfifo->in == pfifo->out));
}

/**
  * @brief          检查环形缓冲区是否已满
  * @param[in]      pfifo: 存储数据的环形缓冲区
  * @retval         如果满了就返回1
  * @attention
  */
unsigned int fifo_is_full(fifo_rx_def *pfifo)
{
    return ((pfifo->size == 0) || (pfifo->size == (pfifo->in - pfifo->out)));
}
