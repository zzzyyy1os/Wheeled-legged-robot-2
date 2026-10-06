#ifndef __SPI_COMM_H__
#define __SPI_COMM_H__

#include "stm32f4xx_hal.h"

/* SPI2 引脚定义 (用户指定) */
#define SPI2_SCK_PIN        GPIO_PIN_13
#define SPI2_SCK_PORT       GPIOB
#define SPI2_MISO_PIN       GPIO_PIN_2
#define SPI2_MISO_PORT      GPIOC
#define SPI2_MOSI_PIN       GPIO_PIN_3
#define SPI2_MOSI_PORT      GPIOC
#define SPI2_CS_PIN         GPIO_PIN_12
#define SPI2_CS_PORT        GPIOB

/* SPI3 引脚定义 (默认引脚) */
#define SPI3_SCK_PIN        GPIO_PIN_3
#define SPI3_SCK_PORT       GPIOB
#define SPI3_MISO_PIN       GPIO_PIN_4
#define SPI3_MISO_PORT      GPIOB
#define SPI3_MOSI_PIN       GPIO_PIN_5
#define SPI3_MOSI_PORT      GPIOB
#define SPI3_CS_PIN         GPIO_PIN_15
#define SPI3_CS_PORT        GPIOA

/* SPI句柄 */
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

/* 函数声明 */
void SPI2_Init(void);
void SPI3_Init(void);
void SPI2_CS_Low(void);
void SPI2_CS_High(void);
void SPI3_CS_Low(void);
void SPI3_CS_High(void);
HAL_StatusTypeDef SPI2_Transmit(uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI2_Receive(uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI2_TransmitReceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI3_Transmit(uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI3_Receive(uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI3_TransmitReceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout);

#endif /* __SPI_COMM_H__ */
