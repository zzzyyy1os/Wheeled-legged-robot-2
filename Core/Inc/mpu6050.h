#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f4xx_hal.h"

/* MPU6050 I2C地址 */
#define MPU6050_WRITE_ADDRESS  0xD0
#define MPU6050_READ_ADDRESS   0xD1

/* 寄存器地址 */
#define SMPLRT_DIV_REG   0x19
#define CONFIG           0x1A
#define GYRO_CONFIG_REG  0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG   0x41
#define GYRO_XOUT_H_REG  0x43
#define PWR_MGMT_1_REG   0x6B
#define PWR_MGMT_2_REG   0x6C
#define WHO_AM_I_REG     0x75

/* MPU6050数据结构体 */
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Ax;       /* 加速度 (g) */
    float Ay;
    float Az;
    float Gx;       /* 角速度 (deg/s) */
    float Gy;
    float Gz;
    float roll_a;   /* 加速度计欧拉角 */
    float pitch_a;
    float roll_g;   /* 陀螺仪欧拉角 */
    float pitch_g;
    float yaw_g;
    float roll;     /* 互补滤波融合结果 */
    float pitch;
    float yaw;
} MPU6050_Data;

/* 函数声明 */
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Result(void);

/* 全局数据 */
extern MPU6050_Data mpu6050_data;

#endif /* __MPU6050_H__ */
