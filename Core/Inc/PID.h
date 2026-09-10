/**
  * @file    PID.h
  * @brief   PID 控制器 (移植自 DengFOC, 适配 STM32 HAL)
  */
#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct {
    float P;                // 比例增益
    float I;                // 积分增益
    float D;                // 微分增益
    float output_ramp;      // 输出变化率限幅 (防止突变)
    float limit;            // 输出限幅

    float error_prev;       // 上一次误差
    float output_prev;      // 上一次输出
    float integral_prev;    // 积分累积值
    uint32_t timestamp_prev;// 上一次执行时间戳 (ms)
} PID_HandleTypeDef;

/**
  * @brief  初始化PID控制器
  * @param  hpid   PID句柄
  * @param  P      比例增益
  * @param  I      积分增益
  * @param  D      微分增益
  * @param  ramp   输出变化率限幅 (output/s), 0=不限制
  * @param  limit  输出限幅
  */
void PID_Init(PID_HandleTypeDef *hpid, float P, float I, float D, float ramp, float limit);

/**
  * @brief  PID计算
  * @param  hpid    PID句柄
  * @param  error   误差值 (目标 - 实际)
  * @retval PID输出
  */
float PID_Update(PID_HandleTypeDef *hpid, float error);

#endif /* __PID_H__ */
