#include "spi_comm.h"
#include "main.h"

/* SPI句柄 */
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/**
 * @brief SPI2初始化 (SCK=PB13, MISO=PC2, MOSI=PC3, CS=PB12)
 */
void SPI2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能时钟 */
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* SPI2 SCK (PB13) */
    GPIO_InitStruct.Pin = SPI2_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(SPI2_SCK_PORT, &GPIO_InitStruct);

    /* SPI2 MISO (PC2) */
    GPIO_InitStruct.Pin = SPI2_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(SPI2_MISO_PORT, &GPIO_InitStruct);

    /* SPI2 MOSI (PC3) */
    GPIO_InitStruct.Pin = SPI2_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(SPI2_MOSI_PORT, &GPIO_InitStruct);

    /* SPI2 CS (PB12) - 软件控制 */
    GPIO_InitStruct.Pin = SPI2_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI2_CS_PORT, &GPIO_InitStruct);

    /* CS默认高电平 (未选中) */
    SPI2_CS_High();

    /* SPI2配置 */
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI3初始化 (SCK=PB3, MISO=PB4, MOSI=PB5, CS=PA15)
 */
void SPI3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能时钟 */
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* SPI3 SCK (PB3) */
    GPIO_InitStruct.Pin = SPI3_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(SPI3_SCK_PORT, &GPIO_InitStruct);

    /* SPI3 MISO (PB4) */
    GPIO_InitStruct.Pin = SPI3_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(SPI3_MISO_PORT, &GPIO_InitStruct);

    /* SPI3 MOSI (PB5) */
    GPIO_InitStruct.Pin = SPI3_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(SPI3_MOSI_PORT, &GPIO_InitStruct);

    /* SPI3 CS (PA15) - 软件控制 */
    GPIO_InitStruct.Pin = SPI3_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI3_CS_PORT, &GPIO_InitStruct);

    /* CS默认高电平 (未选中) */
    SPI3_CS_High();

    /* SPI3配置 */
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI2 CS控制
 */
void SPI2_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI2_CS_PORT, SPI2_CS_PIN, GPIO_PIN_RESET);
}

void SPI2_CS_High(void)
{
    HAL_GPIO_WritePin(SPI2_CS_PORT, SPI2_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief SPI3 CS控制
 */
void SPI3_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI3_CS_PORT, SPI3_CS_PIN, GPIO_PIN_RESET);
}

void SPI3_CS_High(void)
{
    HAL_GPIO_WritePin(SPI3_CS_PORT, SPI3_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief SPI2数据传输函数
 */
HAL_StatusTypeDef SPI2_Transmit(uint8_t *data, uint16_t size, uint32_t timeout)
{
    return HAL_SPI_Transmit(&hspi2, data, size, timeout);
}

HAL_StatusTypeDef SPI2_Receive(uint8_t *data, uint16_t size, uint32_t timeout)
{
    return HAL_SPI_Receive(&hspi2, data, size, timeout);
}

HAL_StatusTypeDef SPI2_TransmitReceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    return HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, size, timeout);
}

/**
 * @brief SPI3数据传输函数
 */
HAL_StatusTypeDef SPI3_Transmit(uint8_t *data, uint16_t size, uint32_t timeout)
{
    return HAL_SPI_Transmit(&hspi3, data, size, timeout);
}

HAL_StatusTypeDef SPI3_Receive(uint8_t *data, uint16_t size, uint32_t timeout)
{
    return HAL_SPI_Receive(&hspi3, data, size, timeout);
}

HAL_StatusTypeDef SPI3_TransmitReceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    return HAL_SPI_TransmitReceive(&hspi3, tx_data, rx_data, size, timeout);
}
