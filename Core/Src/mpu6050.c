/**
  * @file    mpu6050.c
  * @brief   MPU6050 六轴传感器驱动 (I2C1, PB6=SCL, PB7=SDA)
  *          互补滤波融合加速度计和陀螺仪数据
  * @note    移植自TTT项目, 适配STM32F407
  */
#include "mpu6050.h"
#include "i2c.h"
#include "usart.h"
#include <stdint.h>
#include <math.h>

MPU6050_Data mpu6050_data;

/**
 * @brief  MPU6050初始化
 *         验证WHO_AM_I寄存器, 配置采样率、滤波、量程
 */
void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    /* 读取WHO_AM_I寄存器, 验证芯片ID */
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_READ_ADDRESS, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"MPU6050 is ok\r\n", 15, HAL_MAX_DELAY);

        /* 电源管理1: PLL with X-axis gyro */
        Data = 0x01;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDRESS, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        /* 电源管理2 */
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDRESS, PWR_MGMT_2_REG, 1, &Data, 1, 1000);

        /* 低通滤波: 0x06 */
        Data = 0x06;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDRESS, CONFIG, 1, &Data, 1, 1000);

        /* 采样率分频: 0x09 (1kHz / (9+1) = 100Hz) */
        Data = 0x09;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDRESS, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        /* 加速度量程: +/-16g */
        Data = 0x18;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDRESS, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

        /* 陀螺仪量程: +/-2000 dps */
        Data = 0x18;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDRESS, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
    }
    else
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"MPU6050 fail!\r\n", 15, HAL_MAX_DELAY);
    }
}

/**
 * @brief  读取加速度计原始数据并转换为g单位
 *         +/-16g量程, 灵敏度 2048 LSB/g
 */
void MPU6050_Read_Accel(void)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_READ_ADDRESS, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
    mpu6050_data.Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    mpu6050_data.Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    mpu6050_data.Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    mpu6050_data.Ax = mpu6050_data.Accel_X_RAW / 2048.0f;
    mpu6050_data.Ay = mpu6050_data.Accel_Y_RAW / 2048.0f;
    mpu6050_data.Az = mpu6050_data.Accel_Z_RAW / 2048.0f;
}

/**
 * @brief  读取陀螺仪原始数据并转换为deg/s单位
 *         +/-2000 dps量程, 灵敏度 16.4 LSB/(deg/s)
 */
void MPU6050_Read_Gyro(void)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_READ_ADDRESS, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
    mpu6050_data.Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    mpu6050_data.Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    mpu6050_data.Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    mpu6050_data.Gx = mpu6050_data.Gyro_X_RAW / 16.4f;
    mpu6050_data.Gy = mpu6050_data.Gyro_Y_RAW / 16.4f;
    mpu6050_data.Gz = mpu6050_data.Gyro_Z_RAW / 16.4f;
}

/**
 * @brief  读取数据并通过互补滤波融合
 *         调用此函数即可获得 roll, pitch, yaw
 * @note   dt = 0.005 对应 200Hz 采样率 (任务5ms周期)
 */
void MPU6050_Read_Result(void)
{
    const float dt = 0.005f;   /* 采样周期 5ms = 200Hz */
    const float alpha = 0.98f; /* 互补滤波系数 */

    /* 读取加速度 */
    uint8_t Rec_Data_A[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_READ_ADDRESS, ACCEL_XOUT_H_REG, 1, Rec_Data_A, 6, 1000);
    mpu6050_data.Accel_X_RAW = (int16_t)(Rec_Data_A[0] << 8 | Rec_Data_A[1]);
    mpu6050_data.Accel_Y_RAW = (int16_t)(Rec_Data_A[2] << 8 | Rec_Data_A[3]);
    mpu6050_data.Accel_Z_RAW = (int16_t)(Rec_Data_A[4] << 8 | Rec_Data_A[5]);
    mpu6050_data.Ax = mpu6050_data.Accel_X_RAW / 2048.0f;
    mpu6050_data.Ay = mpu6050_data.Accel_Y_RAW / 2048.0f;
    mpu6050_data.Az = mpu6050_data.Accel_Z_RAW / 2048.0f;

    /* 读取陀螺仪 */
    uint8_t Rec_Data_G[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_READ_ADDRESS, GYRO_XOUT_H_REG, 1, Rec_Data_G, 6, 1000);
    mpu6050_data.Gyro_X_RAW = (int16_t)(Rec_Data_G[0] << 8 | Rec_Data_G[1]);
    mpu6050_data.Gyro_Y_RAW = (int16_t)(Rec_Data_G[2] << 8 | Rec_Data_G[3]);
    mpu6050_data.Gyro_Z_RAW = (int16_t)(Rec_Data_G[4] << 8 | Rec_Data_G[5]);
    mpu6050_data.Gx = mpu6050_data.Gyro_X_RAW / 16.4f;
    mpu6050_data.Gy = mpu6050_data.Gyro_Y_RAW / 16.4f;
    mpu6050_data.Gz = mpu6050_data.Gyro_Z_RAW / 16.4f;

    /* 加速度计算欧拉角 */
    mpu6050_data.roll_a  = atan2f(mpu6050_data.Ay, mpu6050_data.Az) / 3.14f * 180.0f;
    mpu6050_data.pitch_a = -atan2f(mpu6050_data.Ax, mpu6050_data.Az) / 3.14f * 180.0f;

    /* 陀螺仪积分 */
    mpu6050_data.roll_g  = mpu6050_data.roll  + mpu6050_data.Gx * dt;
    mpu6050_data.pitch_g = mpu6050_data.pitch + mpu6050_data.Gy * dt;
    mpu6050_data.yaw_g   = mpu6050_data.yaw   + mpu6050_data.Gz * dt;

    /* 互补滤波融合 */
    mpu6050_data.roll  = mpu6050_data.roll  + (mpu6050_data.roll_a  - mpu6050_data.roll_g)  * alpha;
    mpu6050_data.pitch = mpu6050_data.pitch + (mpu6050_data.pitch_a - mpu6050_data.pitch_g) * alpha;
    mpu6050_data.yaw   = mpu6050_data.yaw_g;
}
