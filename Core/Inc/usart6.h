#ifndef __USART6_H__
#define __USART6_H__

#include "main.h"
#include <stdint.h>

/* USART6 句柄 */
extern UART_HandleTypeDef huart6;

/* 单字节接收缓冲 (错误恢复时需要访问) */
extern uint8_t rx_byte;

/* 初始化USART6 (PC6=TX, PC7=RX, 1Mbps, 中断接收) */
void MX_USART6_UART_Init(void);

/* 发送字符串 (阻塞) */
void USART6_SendString(const char *str);

#endif /* __USART6_H__ */
