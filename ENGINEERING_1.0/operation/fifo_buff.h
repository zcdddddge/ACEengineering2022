/**
  ******************************************************************************
  * @file       nepqiu_fifo_buff.c/h
  * @brief      无锁队列
  * @note       v1.0 试用版本
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
  ******************************************************************************
  */

#ifndef __NEPQIU_FIFO_BUFF_H
#define __NEPQIU_FIFO_BUFF_H
#include <string.h>
#include <stdint.h>
#include "struct_typedef.h"

#define FIFO_DMA_ERROR_RX_NOT_IDLE (0x1 << 0)   // 非空闲中断
#define FIFO_DMA_ERROR_RX_POINT_NULL (0x1 << 1) // 指针为空
#define FIFO_DMA_ERROR_RX_FULL (0x1 << 2)       // 非空闲中断

typedef struct
{
    uint8_t *buffer;
    uint32_t in;
    uint32_t out;
    uint16_t size;
    
    uint16_t error; // 接收错误
    uint16_t last_cnt;
} fifo_rx_def;



int32_t fifo_init(fifo_rx_def *pfifo, uint8_t *buff, uint32_t size);
uint32_t fifo_read_buff(fifo_rx_def *pfifo, uint8_t *buffer, uint32_t len);
uint32_t fifo_write_buff(fifo_rx_def *pfifo, uint8_t *buffer, uint32_t len);
unsigned int fifo_get_free(fifo_rx_def *pfifo);
unsigned int fifo_get_full(fifo_rx_def *pfifo);
unsigned int fifo_is_empty(fifo_rx_def *pfifo);
unsigned int fifo_is_full(fifo_rx_def *pfifo);

#endif /* __NEPQIU_FIFO_BUFF_H */
