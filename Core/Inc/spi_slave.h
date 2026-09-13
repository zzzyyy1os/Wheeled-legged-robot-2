/**
  * @file    spi_slave.h
  * @brief   SPI2 从机通信模块 (匹配F103-N主机协议)
  *
  * 帧格式 (8字节固定长度):
  *   [0]    帧头 0xAA
  *   [1]    命令字节 CMD
  *   [2..5] 数据负载 (4字节, 大端序)
  *   [6]    校验: CMD ^ Data[0..3] 的异或值
  *   [7]    帧尾 0x55
  *
  * 接线 (F103-N主机 → F407从机):
  *   PA5 SPI1_SCK  → PB13 SPI2_SCK
  *   PA7 SPI1_MOSI → PB15 SPI2_MOSI
  *   PA6 SPI1_MISO ← PB14 SPI2_MISO
  *   PA4 CS        → PB12 SPI2_NSS
  *   GND           → GND (必须共地)
  */
#ifndef __SPI_SLAVE_H__
#define __SPI_SLAVE_H__

#include "main.h"
#include "cmsis_os.h"

/* 帧参数 (与F103-N一致) */
#define SPI_FRAME_HEAD      0xAA
#define SPI_FRAME_TAIL      0x55
#define SPI_FRAME_LEN       8

/* 命令字 (与F103-N一致 + 电机扩展) */
#define SPI_CMD_HEARTBEAT   0x01    /* 心跳 */
#define SPI_CMD_MPU_DATA    0x02    /* MPU6050数据 */
#define SPI_CMD_M1_VEL      0x03    /* M1目标速度 (float×10, 有符号int32) */
#define SPI_CMD_M2_VEL      0x04    /* M2目标速度 (float×10, 有符号int32) */
#define SPI_CMD_ACK         0x80    /* 从机应答 */

/* 从机应答payload: 上报M1实际速度 (float×10) */
#define SPI_RSP_M1_SPEED    0x01
/* 从机应答payload: 上报M2实际速度 (float×10) */
#define SPI_RSP_M2_SPEED    0x02

/* 接收队列元素 */
typedef struct {
    uint8_t  cmd;
    int32_t  data;      /* 有符号, 因为速度可负 */
} SPI_RxItem_t;

/* 全局句柄 */
extern SPI_HandleTypeDef hspi2;
extern osMessageQueueId_t spiRxQueueHandle;

/* 初始化SPI2从机 (GPIO + 中断, HAL_SPI_MspInit自动调用) */
void MX_SPI2_Slave_Init(void);

/* 初始化SPI2从机 + 创建队列 + 启动监听 */
void SPI_Slave_Init(void);

/* 获取应答数据 (供SPI任务构建回复帧) */
uint32_t SPI_Slave_GetResponse(uint8_t cmd);

/* 等待接收 + 校验 + 解析 + 入队 (由SPI任务调用) */
uint8_t SPI_Slave_WaitAndProcess(void);

#endif /* __SPI_SLAVE_H__ */
