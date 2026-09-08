#ifndef __UART_COMM_H__
#define __UART_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define UART_RX_BUF_SIZE  128
#define UART_TX_BUF_SIZE  256

/* 初始化UART通信, 启动DMA接收 */
void UART_Comm_Init(void);

/* 发送字符串 (阻塞) */
void UART_SendString(const char *str);

/* 格式化发送 (阻塞) */
void UART_Printf(const char *fmt, ...);

/* 获取接收到的数据, 返回长度, 0=无数据 */
uint16_t UART_GetReceivedData(uint8_t *buf, uint16_t max_len);

/* 处理UART接收到的命令 (回显+电机控制) */
void UART_ProcessCommand(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_COMM_H__ */
