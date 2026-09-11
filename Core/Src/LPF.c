/**
  * @file    LPF.c
  * @brief   一阶低通滤波器 (V3P风格, 适配F407)
  */
#include "LPF.h"

static uint32_t Last_Timestamp = 0;
static float Last_y = 0.0f;
static uint8_t lpf_initialized = 0;

void LPF_Init(LPF_HandleTypeDef *hlpf, float Tf)
{
    (void)hlpf; (void)Tf;
    Last_Timestamp = 0;
    Last_y = 0.0f;
    lpf_initialized = 0;
}

float Lowpassfilter(float Tf, float x)
{
    uint32_t now = HAL_GetTick();

    /* 首次调用, 只记录不滤波 */
    if (!lpf_initialized)
    {
        Last_y = x;
        Last_Timestamp = now;
        lpf_initialized = 1;
        return x;
    }

    float dt = (now - Last_Timestamp) * 1e-3f;  /* ms → s */
    Last_Timestamp = now;

    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.5f)
    {
        /* 长时间未调用, 直接返回 */
        Last_y = x;
        return x;
    }

    float alpha = Tf / (Tf + dt);
    float y = alpha * Last_y + (1.0f - alpha) * x;

    Last_y = y;
    return y;
}

/* 保留旧接口兼容 */
float LPF_Update(LPF_HandleTypeDef *hlpf, float x)
{
    (void)hlpf;
    return Lowpassfilter(0.1f, x);
}
