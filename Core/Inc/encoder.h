/**
  * @file    encoder.h
  * @brief   AS5600 编码器数据处理模块
  *          角度归一化、多圈追踪、速度估算
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 编码器句柄 */
typedef struct {
    float angle_latest;           /* 最新单圈角度 (rad, 0~2π) */
    float angle_raw;              /* 滤波后单圈角度 (rad) */
    float angle_prev;             /* 上一次单圈角度 (rad) */
    float full_angle_raw;         /* 多圈累积角度 (rad) */
    float velocity;               /* 估算速度 (rad/s) */
    float vel_angle_prev;         /* 速度计算用的角度 (rad) */
    float vel_full_rotations_prev;/* 速度计算用的圈数 */
    uint32_t vel_timestamp_prev;  /* 速度计算时间戳 (ms) */
} Encoder_HandleTypeDef;

extern Encoder_HandleTypeDef g_encoder;

/* 初始化编码器模块 */
void encoder_init(void);

/* 复位编码器数据 */
void encoder_reset(void);

/* 从 AS5600 更新编码器数据 (每 1ms 调用一次) */
void encoder_update(void);

/* 获取当前角度 (rad, 多圈累积) */
float encoder_get_angle(void);

/* 获取当前速度 (rad/s) */
float encoder_get_velocity(void);

#endif /* __ENCODER_H */
