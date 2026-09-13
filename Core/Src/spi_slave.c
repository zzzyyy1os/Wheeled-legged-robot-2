/**
  * @file    spi_slave.c
  * @brief   SPI2 从机通信 (匹配F103-N主机协议)
  *          SPI2: PB12=NSS, PB13=SCK, PB14=MISO, PB15=MOSI
  *          中断接收, 收到完整帧后送FreeRTOS队列
  */
#include "spi_slave.h"
#include "DengFOC.h"
#include <string.h>

SPI_HandleTypeDef hspi2;

/* 收发缓冲 */
static uint8_t spi_rx_buf[SPI_FRAME_LEN];
static uint8_t spi_tx_buf[SPI_FRAME_LEN];

/* 接收队列 */
osMessageQueueId_t spiRxQueueHandle = NULL;

/* 内部: 等待接收完成信号量 */
static osSemaphoreId_t spi_rx_sem = NULL;
static volatile uint8_t spi_rx_done = 0;

/* ======================== 帧工具函数 ======================== */

static void SPI_PutU32BE(uint8_t *buf, uint8_t offset, uint32_t val)
{
    buf[offset + 0] = (uint8_t)(val >> 24);
    buf[offset + 1] = (uint8_t)(val >> 16);
    buf[offset + 2] = (uint8_t)(val >> 8);
    buf[offset + 3] = (uint8_t)(val);
}

static uint32_t SPI_GetU32BE(const uint8_t *buf, uint8_t offset)
{
    return ((uint32_t)buf[offset + 0] << 24) |
           ((uint32_t)buf[offset + 1] << 16) |
           ((uint32_t)buf[offset + 2] << 8)  |
           ((uint32_t)buf[offset + 3]);
}

static void SPI_BuildFrame(uint8_t *buf, uint8_t cmd, uint32_t data)
{
    buf[0] = SPI_FRAME_HEAD;
    buf[1] = cmd;
    SPI_PutU32BE(buf, 2, data);
    buf[6] = buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
    buf[7] = SPI_FRAME_TAIL;
}

static uint8_t SPI_ValidateFrame(const uint8_t *buf)
{
    if (buf[0] != SPI_FRAME_HEAD || buf[7] != SPI_FRAME_TAIL)
        return 0;
    uint8_t xor_val = buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5];
    return (xor_val == buf[6]) ? 1 : 0;
}

/* ======================== SPI2初始化 ======================== */

void MX_SPI2_Slave_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_SLAVE;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;       /* CPOL=0 */
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;           /* CPHA=0 */
    hspi2.Init.NSS = SPI_NSS_HARD_INPUT;             /* 硬件NSS (PB12) */
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  SPI2 MspInit (GPIO + 中断) - HAL回调
  *         由 HAL_SPI_Init() 自动调用
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (spiHandle->Instance == SPI2)
    {
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB13 = SPI2_SCK (输入, 从机) */
        /* PB15 = SPI2_MOSI (输入, 从机接收) */
        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* PB14 = SPI2_MISO (复用推挽输出, 从机发送) */
        GPIO_InitStruct.Pin = GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* PB12 = SPI2_NSS (硬件输入, 默认高, CS低选中) */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI2 中断 (优先级6, 低于FreeRTOS阈值5) */
        HAL_NVIC_SetPriority(SPI2_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(SPI2_IRQn);
    }
}

/* ======================== SPI回调 ======================== */

/**
  * @brief  SPI全双工收发完成回调 (8字节收发完成)
  *         注意: HAL_SPI_TransmitReceive_IT 完成时调用此回调
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        spi_rx_done = 1;
        if (spi_rx_sem != NULL)
            osSemaphoreRelease(spi_rx_sem);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        /* 出错时重新启动全双工收发 */
        SPI_BuildFrame(spi_tx_buf, SPI_CMD_ACK, 0);
        HAL_SPI_TransmitReceive_IT(&hspi2, spi_tx_buf, spi_rx_buf, SPI_FRAME_LEN);
    }
}

/* ======================== 公共接口 ======================== */

/**
  * @brief  初始化SPI2从机 + 创建队列 + 启动中断接收
  */
void SPI_Slave_Init(void)
{
    /* 硬件初始化 (HAL_SPI_Init 自动调用 HAL_SPI_MspInit) */
    MX_SPI2_Slave_Init();

    /* 创建信号量和队列 */
    if (spi_rx_sem == NULL)
        spi_rx_sem = osSemaphoreNew(1, 0, NULL);

    if (spiRxQueueHandle == NULL)
        spiRxQueueHandle = osMessageQueueNew(8, sizeof(SPI_RxItem_t), NULL);

    /* 清空缓冲 */
    memset(spi_rx_buf, 0, SPI_FRAME_LEN);
    memset(spi_tx_buf, 0, SPI_FRAME_LEN);

    /* 预填充发送帧 (ACK + 0) */
    SPI_BuildFrame(spi_tx_buf, SPI_CMD_ACK, 0);

    /* 启动第一次全双工中断收发 (从机预填充ACK帧) */
    spi_rx_done = 0;
    HAL_SPI_TransmitReceive_IT(&hspi2, spi_tx_buf, spi_rx_buf, SPI_FRAME_LEN);
}

/**
  * @brief  根据命令类型获取应答数据
  */
uint32_t SPI_Slave_GetResponse(uint8_t cmd)
{
    (void)cmd;
    /* 返回M1实际速度 (×10转int32) */
    return (uint32_t)(int32_t)(vel_actual_speed * 10.0f);
}

/**
  * @brief  等待SPI接收完成, 解析帧, 送入队列
  *         由SPI任务循环调用
  * @retval 1=收到有效帧, 0=超时或校验失败
  */
uint8_t SPI_Slave_WaitAndProcess(void)
{
    /* 等待接收完成 (最多50ms, 即5个通信周期) */
    if (osSemaphoreAcquire(spi_rx_sem, pdMS_TO_TICKS(50)) != osOK)
        return 0;

    /* 校验帧 */
    if (!SPI_ValidateFrame(spi_rx_buf))
    {
        /* 帧无效, 重新启动接收 */
        SPI_BuildFrame(spi_tx_buf, SPI_CMD_ACK, 0);
        HAL_SPI_TransmitReceive_IT(&hspi2, spi_tx_buf, spi_rx_buf, SPI_FRAME_LEN);
        return 0;
    }

    /* 解析命令 */
    uint8_t cmd = spi_rx_buf[1];
    int32_t data = (int32_t)SPI_GetU32BE(spi_rx_buf, 2);

    /* 送入队列 (非阻塞) */
    SPI_RxItem_t item = { .cmd = cmd, .data = data };
    osMessageQueuePut(spiRxQueueHandle, &item, 0, 0);

    /* 准备下一帧的应答数据 (M1实际速度) */
    uint32_t rsp_payload = SPI_Slave_GetResponse(cmd);
    SPI_BuildFrame(spi_tx_buf, SPI_CMD_ACK, rsp_payload);

    /* 重新启动全双工中断收发 */
    spi_rx_done = 0;
    HAL_SPI_TransmitReceive_IT(&hspi2, spi_tx_buf, spi_rx_buf, SPI_FRAME_LEN);

    return 1;
}
