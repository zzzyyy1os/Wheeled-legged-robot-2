/**
  * @file    AS5600_M2.c
  * @brief   M2编码器AS5600驱动 (I2C2) + I2C回调路由
  */
#include "AS5600_M2.h"
#include "AS5600.h"
#include "i2c.h"
#include "cmsis_os.h"

#define AS5600_I2C_ADDR         (0x36 << 1)
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_I2C_TIMEOUT      10
#define AS5600_RETRY            3

/* M2全局角度变量 */
volatile float    as5600_m2_angle        = 0.0f;
volatile float    as5600_m2_angle_single = 0.0f;
volatile uint16_t as5600_m2_raw          = 0;
volatile uint8_t  as5600_m2_error        = 0;
volatile uint8_t  as5600_m2_ready        = 0;

/* M2内部状态 */
static float    m2_angle_prev     = 0.0f;
static int32_t  m2_full_rotations = 0;

/* M2中断方式相关 */
static osSemaphoreId_t i2c_m2_sem = NULL;
static volatile uint8_t i2c_m2_result = 0;

/* M1信号量引用 (来自AS5600.c) */
extern osSemaphoreId_t i2c_m1_sem;
extern volatile uint8_t i2c_m1_result;

/* 前向声明 */
static uint8_t AS5600_M2_GetRawAngle(uint16_t *out);

/* ======================== I2C中断回调路由 ======================== */
/* 将所有I2C回调集中在此文件, 根据Instance路由到M1/M2 */

/**
  * @brief  I2C 内存读取完成回调 (中断模式)
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        /* M1 AS5600 */
        i2c_m1_result = 1;
        osSemaphoreRelease(i2c_m1_sem);
    }
    else if (hi2c->Instance == I2C2)
    {
        /* M2 AS5600 */
        i2c_m2_result = 1;
        osSemaphoreRelease(i2c_m2_sem);
    }
}

/**
  * @brief  I2C 错误回调
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        i2c_m1_result = 0;
        osSemaphoreRelease(i2c_m1_sem);
    }
    else if (hi2c->Instance == I2C2)
    {
        i2c_m2_result = 0;
        osSemaphoreRelease(i2c_m2_sem);
    }
}

/* ======================== M2总线恢复 ======================== */

void AS5600_M2_BusRecovery(void)
{
    HAL_I2C_DeInit(&hi2c2);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* PB10 = I2C2_SCL */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB11 = I2C2_SDA */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET)
            break;
    }

    HAL_I2C_Init(&hi2c2);
    HAL_Delay(2);
}

/* ======================== M2初始化 ======================== */

void AS5600_M2_Init(void)
{
    m2_angle_prev = 0.0f;
    m2_full_rotations = 0;
    as5600_m2_error = 0;
    as5600_m2_ready = 0;

    /* 创建信号量 */
    if (i2c_m2_sem == NULL)
    {
        i2c_m2_sem = osSemaphoreNew(1, 0, NULL);
    }

    /* 使能I2C2中断 (优先级6, 低于FreeRTOS阈值5) */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

    /* 尝试读取一次 (阻塞方式检测传感器) */
    uint16_t raw;
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H,
                          I2C_MEMADD_SIZE_8BIT, buf, 2, 50) == HAL_OK)
    {
        raw = ((uint16_t)buf[0] << 8) | buf[1];
        as5600_m2_ready = 1;
        as5600_m2_raw = raw;
        as5600_m2_angle_single = (float)raw * 0.08789f * 3.14159265f / 180.0f;
        m2_angle_prev = as5600_m2_angle_single;
        as5600_m2_angle = as5600_m2_angle_single;
    }
}

/* ======================== M2中断读取 ======================== */

static uint8_t AS5600_M2_GetRawAngle(uint16_t *out)
{
    uint8_t buf[2];

    for (int retry = 0; retry <= AS5600_RETRY; retry++)
    {
        /* 清空信号量 */
        while (osSemaphoreAcquire(i2c_m2_sem, 0) == osOK) {}
        i2c_m2_result = 0;

        /* 启动中断读取 */
        if (HAL_I2C_Mem_Read_IT(&hi2c2, AS5600_I2C_ADDR,
                                 AS5600_REG_RAW_ANGLE_H, I2C_MEMADD_SIZE_8BIT,
                                 buf, 2) != HAL_OK)
        {
            HAL_I2C_DeInit(&hi2c2);
            HAL_I2C_Init(&hi2c2);
            HAL_Delay(1);
            continue;
        }

        /* 等待完成 (最多10ms) */
        if (osSemaphoreAcquire(i2c_m2_sem, pdMS_TO_TICKS(10)) == osOK && i2c_m2_result)
        {
            *out = ((uint16_t)buf[0] << 8) | buf[1];
            return 1;
        }

        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_Delay(1);
    }

    return 0;
}

/* ======================== M2公共接口 ======================== */

uint8_t AS5600_M2_Read(void)
{
    uint16_t raw;
    if (!AS5600_M2_GetRawAngle(&raw))
    {
        as5600_m2_error++;
        if (as5600_m2_error >= 3)
        {
            AS5600_M2_BusRecovery();
            as5600_m2_error = 0;
        }
        return 0;
    }

    as5600_m2_error = 0;
    as5600_m2_raw = raw;

    float val = (float)raw * 0.08789f * 3.14159265f / 180.0f;
    as5600_m2_angle_single = val;

    float d_angle = val - m2_angle_prev;
    if (fabsf(d_angle) > (0.8f * 6.2831853f))
        m2_full_rotations += (d_angle > 0) ? -1 : 1;
    m2_angle_prev = val;
    as5600_m2_angle = (float)m2_full_rotations * 6.2831853f + m2_angle_prev;

    as5600_m2_ready = 1;
    return 1;
}

float GetAngle_M2(void)
{
    return (float)as5600_m2_angle;
}

float GetAngle_NoTrack_M2(void)
{
    return (float)as5600_m2_angle_single;
}

/* M2速度计算 (使用HAL_GetTick, 适配F407) */
static uint32_t M2_Last_Vel_tick = 0;
static float M2_Vel_Last_Angle = 0.0f;
static uint8_t m2_vel_initialized = 0;

float GetVelocity_M2(void)
{
    uint32_t now = HAL_GetTick();
    float Vel_Angle = GetAngle_M2();

    if (!m2_vel_initialized)
    {
        M2_Vel_Last_Angle = Vel_Angle;
        M2_Last_Vel_tick = now;
        m2_vel_initialized = 1;
        return 0.0f;
    }

    float dt = (now - M2_Last_Vel_tick) * 1e-3f;
    M2_Last_Vel_tick = now;

    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.5f) dt = 0.5f;

    float velocity = (Vel_Angle - M2_Vel_Last_Angle) / dt;

    M2_Vel_Last_Angle = Vel_Angle;

    return velocity;
}
