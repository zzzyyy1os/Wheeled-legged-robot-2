#include "AS5600.h"
#include "i2c.h"

#define AS5600_I2C_ADDR       (0x36 << 1)  // 7-bit address 0x36, shifted for HAL
#define AS5600_REG_RAW_ANGLE_H  0x0C
#define AS5600_REG_RAW_ANGLE_L  0x0D

#define AS5600_I2C_TIMEOUT      10  // ms

static float angle_prev = 0.0f;
static int32_t full_rotations = 0;

/**
 * @brief  Initialize AS5600 sensor
 */
void AS5600_Init(void)
{
    // I2C3 is already initialized in MX_I2C3_Init()
    // Just reset tracking variables
    angle_prev = 0.0f;
    full_rotations = 0;
}

/**
 * @brief  Read two bytes (high/low) from AS5600 register
 */
static uint16_t AS5600_ReadTwoBytes(uint8_t reg_hi, uint8_t reg_lo)
{
    uint8_t buf[2];
    uint16_t retVal = 0;

    // Read low byte
    if (HAL_I2C_Mem_Read(&hi2c3, AS5600_I2C_ADDR, reg_lo,
                          I2C_MEMADD_SIZE_8BIT, &buf[0], 1, AS5600_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    // Read high byte
    if (HAL_I2C_Mem_Read(&hi2c3, AS5600_I2C_ADDR, reg_hi,
                          I2C_MEMADD_SIZE_8BIT, &buf[1], 1, AS5600_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    retVal = ((uint16_t)buf[1] << 8) | buf[0];
    return retVal;
}

/**
 * @brief  Get raw angle (0-4095, 12-bit)
 */
uint16_t AS5600_GetRawAngle(void)
{
    return AS5600_ReadTwoBytes(AS5600_REG_RAW_ANGLE_H, AS5600_REG_RAW_ANGLE_L);
}

/**
 * @brief  Get single-turn angle in radians
 */
float AS5600_GetAngle_Without_Track(void)
{
    return (float)AS5600_GetRawAngle() * 0.08789f * 3.14159265f / 180.0f;
}

/**
 * @brief  Get cumulative angle in radians (with rotation tracking)
 */
float AS5600_GetAngle(void)
{
    float val = AS5600_GetAngle_Without_Track();
    float d_angle = val - angle_prev;

    // Detect overflow: if angle jumped more than 80% of a full revolution
    if (fabsf(d_angle) > (0.8f * 6.2831853f))
    {
        full_rotations += (d_angle > 0) ? -1 : 1;
    }

    angle_prev = val;
    return (float)full_rotations * 6.2831853f + angle_prev;
}
