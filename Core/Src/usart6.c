/**
  * @file    usart6.c
  * @brief   USART6配置 - ESP32通讯, 1Mbps, PC6=TX, PC7=RX
  *          中断单字节接收, 收到换行符后送入uart_rx_queue
  */
#include "usart6.h"
#include "uart_comm.h"
#include "cmsis_os.h"
#include <string.h>

UART_HandleTypeDef huart6;

/* 接收相关 */
uint8_t rx_byte;                       /* 单字节接收缓冲 */
static char cmd_buf[128];              /* 命令行缓冲 */
static volatile uint8_t cmd_idx = 0;   /* 命令索引 */

/**
 * @brief  USART6 GPIO初始化 (PC6=TX, PC7=RX)
 */
static void USART6_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
}

/**
 * @brief  USART6初始化 (1Mbps, 8N1, 中断接收)
 */
void MX_USART6_UART_Init(void)
{
    USART6_GPIO_Init();

    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 1000000;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart6);

    cmd_idx = 0;
    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
}

/**
 * @brief  发送字符串 (阻塞)
 */
void USART6_SendString(const char *str)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)str, strlen(str), 100);
}

/**
 * @brief  USART6中断处理
 */
void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart6);
}

/**
 * @brief  中断回调: 收到一个字节
 *         收到换行符(\n或\r)后将命令送入uart_rx_queue
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        if (rx_byte == '\n' || rx_byte == '\r')
        {
            if (cmd_idx > 0)
            {
                cmd_buf[cmd_idx] = '\0';
                if (uart_rx_queue != NULL)
                {
                    osMessageQueuePut(uart_rx_queue, cmd_buf, 0, 0);
                }
                cmd_idx = 0;
            }
        }
        else
        {
            if (cmd_idx < sizeof(cmd_buf) - 1)
            {
                cmd_buf[cmd_idx++] = rx_byte;
            }
            else
            {
                cmd_idx = 0;  /* 溢出时重置 */
            }
        }

        /* 重新启动接收 */
        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
    }
}
