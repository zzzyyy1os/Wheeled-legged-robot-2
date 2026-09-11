/**
  * @file    LPF.h
  * @brief   一阶低通滤波器 (移植自 DengFOC)
  */
#ifndef __LPF_H__
#define __LPF_H__

#include "main.h"

typedef struct {
    float Tf;           // 低通滤波时间常数 (秒)
    float y_prev;       // 上一次滤波输出值
    uint32_t timestamp_prev; // 上一次执行时间戳 (ms)
} LPF_HandleTypeDef;

/**
  * @brief  初始化低通滤波器
  * @param  hlpf   滤波器句柄
  * @param  Tf     时间常数 (秒), 越大滤波越强
  */
void LPF_Init(LPF_HandleTypeDef *hlpf, float Tf);

/**
  * @brief  低通滤波器计算
  * @param  hlpf   滤波器句柄
  * @param  x      输入信号
  * @retval 滤波后的输出
  */
float LPF_Update(LPF_HandleTypeDef *hlpf, float x);

/**
  * @brief  低通滤波器 (V3P风格, 简洁版)
  * @param  Tf   时间常数 (秒)
  * @param  x    输入信号
  * @retval 滤波后的输出
  */
float Lowpassfilter(float Tf, float x);

#endif /* __LPF_H__ */
