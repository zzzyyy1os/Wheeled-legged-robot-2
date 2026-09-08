#include "uart_comm.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

static uint8_t rx_buf[UART_RX_BUF_SIZE];
static uint8_t tx_buf[UART_TX_BUF_SIZE];

/* 接收完成标志 */
volatile uint8_t uart_rx_flag = 0;
volatile uint16_t uart_rx_len = 0;

/**
 * @brief  初始化UART通信, 启动DMA接收
 */
void UART_Comm_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, UART_RX_BUF_SIZE);
}

/**
 * @brief  发送字符串 (阻塞)
 */
void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}

/**
 * @brief  格式化发送 (阻塞)
 */
void UART_Printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)tx_buf, UART_TX_BUF_SIZE, fmt, ap);
    va_end(ap);
    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, tx_buf, len, 100);
    }
}

/**
 * @brief  UART空闲中断回调 (ISR上下文调用)
 *         收到数据后设置标志, 重新启动DMA接收
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        uart_rx_len = Size;
        uart_rx_flag = 1;
        /* 重新启动DMA接收 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, UART_RX_BUF_SIZE);
    }
}

/**
 * @brief  UART错误回调 - 重新启动DMA接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, UART_RX_BUF_SIZE);
    }
}

/**
 * @brief  获取接收到的数据 (非ISR调用)
 * @param  buf: 输出缓冲区
 * @param  max_len: 最大长度
 * @return 实际拷贝长度, 0表示无数据
 */
uint16_t UART_GetReceivedData(uint8_t *buf, uint16_t max_len)
{
    if (uart_rx_flag == 0) return 0;

    uint16_t copy_len = uart_rx_len;
    if (copy_len > max_len - 1) copy_len = max_len - 1;

    memcpy(buf, rx_buf, copy_len);
    buf[copy_len] = '\0';

    uart_rx_flag = 0;
    uart_rx_len = 0;

    return copy_len;
}
