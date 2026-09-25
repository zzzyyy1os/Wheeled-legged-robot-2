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
volatile float    as5600_velocity     = 0.0f;  /* 预计算速度, 无竞态 */
volatile uint16_t as5600_raw          = 0;
volatile uint8_t  as5600_error        = 0;
volatile uint8_t  as5600_ready        = 0;

/* 内部状态 */
static float    angle_prev     = 0.0f;
static int32_t  full_rotations = 0;
static uint32_t vel_last_us    = 0;  /* 速度计算用, DWT微秒 */
static float    vel_last_angle = 0.0f;
static uint8_t  vel_inited     = 0;

/* 中断方式相关 (非static, 供AS5600_M2.c中的回调路由使用) */
osSemaphoreId_t i2c_m1_sem = NULL;
volatile uint8_t i2c_m1_result = 0;

/* 前向声明 */
static uint8_t AS5600_GetRawAngle(uint16_t *out);

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
    vel_inited = 0;      /* 重置速度计算状态 */
    as5600_velocity = 0.0f;

    /* 创建信号量 */
    if (i2c_m1_sem == NULL)
    {
        i2c_m1_sem = osSemaphoreNew(1, 0, NULL);
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
        while (osSemaphoreAcquire(i2c_m1_sem, 0) == osOK) {}
        i2c_m1_result = 0;

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
        if (osSemaphoreAcquire(i2c_m1_sem, pdMS_TO_TICKS(10)) == osOK && i2c_m1_result)
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

    /* 在同一函数内计算速度, 消除跨任务竞态 */
    {
        uint32_t now_us = DWT->CYCCNT / (SystemCoreClock / 1000000);
        if (!vel_inited)
        {
            vel_last_us = now_us;
            vel_last_angle = as5600_angle;
            vel_inited = 1;
            as5600_velocity = 0.0f;
        }
        else
        {
            float dt = (now_us - vel_last_us) * 1e-6f;
            vel_last_us = now_us;
            if (dt < 0.0001f) dt = 0.0001f;
            if (dt > 0.5f) dt = 0.5f;
            as5600_velocity = (as5600_angle - vel_last_angle) / dt;
            vel_last_angle = as5600_angle;
        }
    }

    as5600_ready = 1;
    return 1;
}

/* ======================== V3P风格接口 ======================== */

/**
  * @brief  获取多圈角度 (兼容V3P GetAngle)
  */
float GetAngle(void)
{
    return (float)as5600_angle;
}

/**
  * @brief  获取单圈角度 (兼容V3P GetAngle_NoTrack)
  */
float GetAngle_NoTrack(void)
{
    return (float)as5600_angle_single;
}

/**
  * @brief  获取速度 (在AS5600_Read()中预计算, 无竞态)
  */
float GetVelocity(void)
{
    return (float)as5600_velocity;
}
