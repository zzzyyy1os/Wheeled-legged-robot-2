#include "AS5600.h"
#include "i2c.h"
#include "cmsis_os.h"

#define AS5600_I2C_ADDR         (0x36 << 1)
#define AS5600_REG_RAW_ANGLE_H  0x0C

#define AS5600_I2C_TIMEOUT      10
#define AS5600_RETRY            3

/* 全局角度变量 */
volatile float    as5600_angle        = 0.0f;
volatile float    as5600_angle_single = 0.0f;
volatile uint16_t as5600_raw          = 0;
volatile uint8_t  as5600_error        = 0;
volatile uint8_t  as5600_ready        = 0;

/* 内部状态 */
static float    angle_prev     = 0.0f;
static int32_t  full_rotations = 0;

/* 中断方式相关 */
static osSemaphoreId_t i2c_sem = NULL;
static volatile uint8_t i2c_result = 0;

/* 前向声明 */
static uint8_t AS5600_GetRawAngle(uint16_t *out);

/* ======================== I2C 中断回调 ======================== */

/**
  * @brief  I2C 内存读取完成回调 (中断模式)
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        i2c_result = 1;
        osSemaphoreRelease(i2c_sem);
    }
}

/**
  * @brief  I2C 错误回调
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        i2c_result = 0;
        osSemaphoreRelease(i2c_sem);
    }
}

/* ======================== 总线恢复 ======================== */

void AS5600_BusRecovery(void)
{
    HAL_I2C_DeInit(&hi2c3);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET)
            break;
    }

    HAL_I2C_Init(&hi2c3);
    HAL_Delay(2);
}

/* ======================== 初始化 ======================== */

void AS5600_Init(void)
{
    angle_prev = 0.0f;
    full_rotations = 0;
    as5600_error = 0;
    as5600_ready = 0;

    /* 创建信号量 */
    if (i2c_sem == NULL)
    {
        i2c_sem = osSemaphoreNew(1, 0, NULL);
    }

    /* 使能I2C3中断 (优先级5, 低于FreeRTOS系统调用阈值) */
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);

    /* 尝试读取一次 (用阻塞方式检测传感器) */
    uint16_t raw;
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c3, AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H,
                          I2C_MEMADD_SIZE_8BIT, buf, 2, 50) == HAL_OK)
    {
        raw = ((uint16_t)buf[0] << 8) | buf[1];
        as5600_ready = 1;
        as5600_raw = raw;
        as5600_angle_single = (float)raw * 0.08789f * 3.14159265f / 180.0f;
        angle_prev = as5600_angle_single;
        as5600_angle = as5600_angle_single;
    }
}

/* ======================== 中断读取 ======================== */

/**
  * @brief  通过中断方式读取AS5600原始角度
  * @param  out: 输出原始值
  * @retval 1=成功, 0=I2C失败
  */
static uint8_t AS5600_GetRawAngle(uint16_t *out)
{
    uint8_t buf[2];

    for (int retry = 0; retry <= AS5600_RETRY; retry++)
    {
        /* 清空信号量 */
        while (osSemaphoreAcquire(i2c_sem, 0) == osOK) {}
        i2c_result = 0;

        /* 启动中断读取 */
        if (HAL_I2C_Mem_Read_IT(&hi2c3, AS5600_I2C_ADDR,
                                 AS5600_REG_RAW_ANGLE_H, I2C_MEMADD_SIZE_8BIT,
                                 buf, 2) != HAL_OK)
        {
            /* 启动失败, 重置I2C */
            HAL_I2C_DeInit(&hi2c3);
            HAL_I2C_Init(&hi2c3);
            HAL_Delay(1);
            continue;
        }

        /* 等待完成 (最多10ms) */
        if (osSemaphoreAcquire(i2c_sem, pdMS_TO_TICKS(10)) == osOK && i2c_result)
        {
            *out = ((uint16_t)buf[0] << 8) | buf[1];
            return 1;
        }

        /* 超时或错误, 重置I2C */
        HAL_I2C_DeInit(&hi2c3);
        HAL_I2C_Init(&hi2c3);
        HAL_Delay(1);
    }

    return 0;
}

/* ======================== 公共接口 ======================== */

float AS5600_GetAngle_Without_Track(void)
{
    uint16_t raw;
    if (!AS5600_GetRawAngle(&raw)) return 0.0f;
    return (float)raw * 0.08789f * 3.14159265f / 180.0f;
}

float AS5600_GetAngle(void)
{
    float val = AS5600_GetAngle_Without_Track();
    float d_angle = val - angle_prev;
    if (fabsf(d_angle) > (0.8f * 6.2831853f))
        full_rotations += (d_angle > 0) ? -1 : 1;
    angle_prev = val;
    return (float)full_rotations * 6.2831853f + angle_prev;
}

uint8_t AS5600_Read(void)
{
    uint16_t raw;
    if (!AS5600_GetRawAngle(&raw))
    {
        as5600_error++;
        if (as5600_error >= 10)
        {
            AS5600_BusRecovery();
            as5600_error = 0;
        }
        return 0;
    }

    as5600_error = 0;
    as5600_raw = raw;

    float val = (float)raw * 0.08789f * 3.14159265f / 180.0f;
    as5600_angle_single = val;

    float d_angle = val - angle_prev;
    if (fabsf(d_angle) > (0.8f * 6.2831853f))
        full_rotations += (d_angle > 0) ? -1 : 1;
    angle_prev = val;
    as5600_angle = (float)full_rotations * 6.2831853f + angle_prev;

    as5600_ready = 1;
    return 1;
}
