#include "uart_comm.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static uint8_t rx_buf[UART_RX_BUF_SIZE];
static uint8_t tx_buf[UART_TX_BUF_SIZE];

/* DMA发送状态: 0=空闲, 1=发送中 */
static volatile uint8_t tx_busy = 0;

/* FreeRTOS消息队列句柄 */
osMessageQueueId_t uart_rx_queue = NULL;

/**
 * @brief  初始化UART通信, 启动DMA接收, 创建消息队列
 */
void UART_Comm_Init(void)
{
    tx_busy = 0;
    uart_rx_queue = osMessageQueueNew(16, UART_RX_BUF_SIZE, NULL);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, UART_RX_BUF_SIZE);
}

/**
 * @brief  发送字符串 (DMA非阻塞, 忙则跳过)
 */
void UART_SendString(const char *str)
{
    if (tx_busy) return;  /* 上一次DMA还没发完, 跳过 */
    tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)str, strlen(str));
}

/**
 * @brief  格式化发送 (DMA非阻塞, 忙则跳过)
 */
void UART_Printf(const char *fmt, ...)
{
    if (tx_busy) return;  /* 上一次DMA还没发完, 跳过 */

    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)tx_buf, UART_TX_BUF_SIZE, fmt, ap);
    va_end(ap);

    if (len > 0)
    {
        tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
    }
}

/**
 * @brief  DMA发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        tx_busy = 0;
    }
}

/**
 * @brief  UART空闲中断回调 (ISR上下文调用)
 *         收到数据后送入FreeRTOS队列, 重新启动DMA接收
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        if (uart_rx_queue != NULL)
        {
            osMessageQueuePut(uart_rx_queue, rx_buf, 0, 0);
        }
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
