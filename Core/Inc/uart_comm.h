#ifndef __UART_COMM_H__
#define __UART_COMM_H__

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

#define UART_RX_BUF_SIZE  128
#define UART_TX_BUF_SIZE  256

/* FreeRTOS消息队列句柄 (MotorTask中用于接收UART命令) */
extern osMessageQueueId_t uart_rx_queue;

/* 初始化UART通信, 启动DMA接收, 创建消息队列 */
void UART_Comm_Init(void);

/* 发送字符串 (阻塞) */
void UART_SendString(const char *str);

/* 格式化发送 (阻塞) */
void UART_Printf(const char *fmt, ...);

#endif /* __UART_COMM_H__ */
