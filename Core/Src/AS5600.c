#include "AS5600.h"
#include "i2c.h"

#define AS5600_I2C_ADDR       (0x36 << 1)  // 7-bit address 0x36, shifted for HAL
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_RAW_ANGLE_L  0x0D
#define AS5600_REG_STATUS       0x0B

#define AS5600_I2C_TIMEOUT      10  // ms

/* 全局角度变量 */
volatile float    as5600_angle        = 0.0f;
volatile float    as5600_angle_single = 0.0f;
volatile uint16_t as5600_raw          = 0;
volatile uint8_t  as5600_error        = 0;
volatile uint8_t  as5600_ready        = 0;

/* 内部状态 */
static float    angle_prev     = 0.0f;
static int32_t  full_rotations = 0;

/* 前向声明 */
static uint8_t AS5600_GetRawAngle(uint16_t *out);

/**
 * @brief  I2C总线恢复 - 发送时钟脉冲释放SDA
 *         当I2C从设备拉住SDA不放时, 主设备发送9个SCL脉冲释放总线
 */
void AS5600_BusRecovery(void)
{
    /* 配置SCL为GPIO输出模式 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 发送9个SCL时钟脉冲 */
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* 恢复I2C复用功能 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 重新初始化I2C3 */
    HAL_I2C_DeInit(&hi2c3);
    HAL_I2C_Init(&hi2c3);
}

/**
 * @brief  初始化AS5600传感器
 */
void AS5600_Init(void)
{
    angle_prev = 0.0f;
    full_rotations = 0;
    as5600_error = 0;
    as5600_ready = 0;

    /* 尝试读取一次, 检测传感器是否存在 */
    uint16_t raw;
    if (AS5600_GetRawAngle(&raw))
    {
        as5600_ready = 1;
        as5600_raw = raw;
        as5600_angle_single = (float)raw * 0.08789f * 3.14159265f / 180.0f;
        angle_prev = as5600_angle_single;
        as5600_angle = as5600_angle_single;
    }
}

/**
 * @brief  读取AS5600原始角度 (0-4095)
 * @param  out: 输出原始值
 * @retval 1=成功, 0=I2C失败
 */
static uint8_t AS5600_GetRawAngle(uint16_t *out)
{
    uint8_t buf[2];

    /* 从高字节寄存器0x0C开始读2字节: [high, low] */
    if (HAL_I2C_Mem_Read(&hi2c3, AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE_H,
                          I2C_MEMADD_SIZE_8BIT, buf, 2, AS5600_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    /* AS5600返回: buf[0]=高字节, buf[1]=低字节 (大端序) */
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return 1;
}

/**
 * @brief  获取单圈角度 (rad)
 */
float AS5600_GetAngle_Without_Track(void)
{
    uint16_t raw;
    if (!AS5600_GetRawAngle(&raw)) return 0.0f;
    return (float)raw * 0.08789f * 3.14159265f / 180.0f;
}

/**
 * @brief  获取累积多圈角度 (rad)
 */
float AS5600_GetAngle(void)
{
    float val = AS5600_GetAngle_Without_Track();

    float d_angle = val - angle_prev;

    if (fabsf(d_angle) > (0.8f * 6.2831853f))
    {
        full_rotations += (d_angle > 0) ? -1 : 1;
    }

    angle_prev = val;
    return (float)full_rotations * 6.2831853f + angle_prev;
}

/**
 * @brief  读取AS5600并更新全局变量 (供AS5600任务调用)
 * @retval 1=成功, 0=失败
 */
uint8_t AS5600_Read(void)
{
    uint16_t raw;
    if (!AS5600_GetRawAngle(&raw))
    {
        /* I2C通信失败 */
        as5600_error++;
        if (as5600_error > 10)
        {
            /* 连续失败10次, 尝试总线恢复 */
            AS5600_BusRecovery();
            as5600_error = 0;
        }
        return 0;
    }

    /* 读取成功 */
    as5600_error = 0;
    as5600_raw = raw;

    /* 单圈角度 */
    float val = (float)raw * 0.08789f * 3.14159265f / 180.0f;
    as5600_angle_single = val;

    /* 累积多圈角度 */
    float d_angle = val - angle_prev;
    if (fabsf(d_angle) > (0.8f * 6.2831853f))
    {
        full_rotations += (d_angle > 0) ? -1 : 1;
    }
    angle_prev = val;
    as5600_angle = (float)full_rotations * 6.2831853f + angle_prev;

    as5600_ready = 1;
    return 1;
}
