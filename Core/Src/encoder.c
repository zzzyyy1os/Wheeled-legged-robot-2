/**
  * @file    encoder.c
  * @brief   AS5600 编码器数据处理模块
  *          角度归一化、多圈追踪、速度估算
  */
#include "encoder.h"
#include "AS5600.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define M_2PI   (2.0f * M_PI)

Encoder_HandleTypeDef g_encoder;

/* 传感器数据有效性标记 */
static uint8_t sensor_valid = 0;

void encoder_init(void)
{
    g_encoder.angle_latest           = 0.0f;
    g_encoder.angle_raw              = 0.0f;
    g_encoder.angle_prev             = 0.0f;
    g_encoder.full_angle_raw         = 0.0f;
    g_encoder.velocity               = 0.0f;
    g_encoder.vel_angle_prev         = 0.0f;
    g_encoder.vel_full_rotations_prev= 0.0f;
    g_encoder.vel_timestamp_prev     = HAL_GetTick();
    sensor_valid = 0;
}

void encoder_reset(void)
{
    g_encoder.angle_latest           = 0.0f;
    g_encoder.angle_raw              = 0.0f;
    g_encoder.angle_prev             = 0.0f;
    g_encoder.full_angle_raw         = 0.0f;
    g_encoder.velocity               = 0.0f;
    g_encoder.vel_angle_prev         = 0.0f;
    g_encoder.vel_full_rotations_prev= 0.0f;
    g_encoder.vel_timestamp_prev     = HAL_GetTick();
    sensor_valid = 0;
}

void encoder_update(void)
{
    /* 检查AS5600是否就绪且有新数据 */
    if (!as5600_ready || as5600_error > 0)
    {
        /* 传感器不可用, 停止速度更新, 保持上一次的值 */
        return;
    }

    /* 读取AS5600单圈角度 (0 ~ 2π) */
    g_encoder.angle_latest = as5600_angle_single;

    float delta_angle = g_encoder.angle_latest - g_encoder.angle_prev;

    /* 过零处理: 跨越 0°/360° 边界 */
    if (delta_angle > M_PI)
    {
        delta_angle -= M_2PI;
    }
    else if (delta_angle < -M_PI)
    {
        delta_angle += M_2PI;
    }

    /* 首次有效数据, 只记录不计算 */
    if (!sensor_valid)
    {
        g_encoder.angle_prev = g_encoder.angle_latest;
        g_encoder.vel_angle_prev = g_encoder.angle_latest;
        g_encoder.vel_full_rotations_prev = 0;
        g_encoder.vel_timestamp_prev = HAL_GetTick();
        sensor_valid = 1;
        return;
    }

    /* 更新多圈累积角度 */
    g_encoder.full_angle_raw += delta_angle;
    g_encoder.angle_prev = g_encoder.angle_latest;

    /* 低通滤波单圈角度 */
    g_encoder.angle_raw = 0.5f * g_encoder.angle_latest + 0.5f * g_encoder.angle_raw;

    /* ---- 速度计算 ---- */
    uint32_t now = HAL_GetTick();
    float Ts = (now - g_encoder.vel_timestamp_prev) * 1e-3f;
    g_encoder.vel_timestamp_prev = now;

    if (Ts <= 0.0f || Ts > 0.5f)
    {
        /* 时间异常, 跳过 */
        g_encoder.vel_angle_prev = g_encoder.angle_latest;
        g_encoder.vel_full_rotations_prev = (int32_t)floorf(g_encoder.full_angle_raw / M_2PI);
        return;
    }

    int32_t full_rotations = (int32_t)floorf(g_encoder.full_angle_raw / M_2PI);
    float angle_prev_total = g_encoder.vel_angle_prev + g_encoder.vel_full_rotations_prev * M_2PI;
    float angle_now_total  = g_encoder.angle_latest + full_rotations * M_2PI;

    float new_velocity = (angle_now_total - angle_prev_total) / Ts;

    /* 速度限幅保护 (防止异常跳变) */
    if (fabsf(new_velocity) > 500.0f)
    {
        /* 速度异常, 不更新 */
        g_encoder.vel_angle_prev = g_encoder.angle_latest;
        g_encoder.vel_full_rotations_prev = full_rotations;
        return;
    }

    /* 一阶低通滤波速度 (加大滤波强度) */
    g_encoder.velocity = 0.3f * new_velocity + 0.7f * g_encoder.velocity;

    g_encoder.vel_angle_prev = g_encoder.angle_latest;
    g_encoder.vel_full_rotations_prev = full_rotations;
}

float encoder_get_angle(void)
{
    return g_encoder.full_angle_raw;
}

float encoder_get_velocity(void)
{
    return g_encoder.velocity;
}
