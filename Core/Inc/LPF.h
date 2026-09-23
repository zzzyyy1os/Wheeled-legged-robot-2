/**
  * @file    LPF.h
  * @brief   一阶低通器 (V3P风格, 适配F407)
  */
#ifndef __LPF_H__
#define __LPF_H__

#include "main.h"

/**
  * @brief  低通滤波器 (全局, M1使用)
  */
float Lowpassfilter(float Tf, float x);

/* ======================== 多实例LPF (双电机支持) ======================== */

typedef struct {
    uint32_t Last_Timestamp;
    float    Last_y;
    uint8_t  initialized;
} LPF_Instance_t;

void LPF_Instance_Init(LPF_Instance_t *inst);
void LPF_Instance_Reset(LPF_Instance_t *inst);
float Lowpassfilter_Instance(LPF_Instance_t *inst, float Tf, float x);

#endif /* __LPF_H__ */
