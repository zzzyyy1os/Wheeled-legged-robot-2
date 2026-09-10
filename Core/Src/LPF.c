/**
  * @file    LPF.c
  * @brief   一阶低通滤波器 (移植自 DengFOC, 适配 STM32 HAL)
  */
#include "LPF.h"

void LPF_Init(LPF_HandleTypeDef *hlpf, float Tf)
{
    hlpf->Tf = Tf;
    hlpf->y_prev = 0.0f;
    hlpf->timestamp_prev = HAL_GetTick();
}

float LPF_Update(LPF_HandleTypeDef *hlpf, float x)
{
    uint32_t timestamp = HAL_GetTick();
    float dt = (timestamp - hlpf->timestamp_prev) * 1e-3f;  /* ms → s */

    /* 时间异常处理 */
    if (dt < 0.0f)
    {
        dt = 1e-3f;
    }
    else if (dt > 0.3f)
    {
        /* 首次调用或长时间未调用, 直接返回输入值 */
        hlpf->y_prev = x;
        hlpf->timestamp_prev = timestamp;
        return x;
    }

    /* 一阶低通滤波: y = alpha * y_prev + (1 - alpha) * x */
    float alpha = hlpf->Tf / (hlpf->Tf + dt);
    float y = alpha * hlpf->y_prev + (1.0f - alpha) * x;

    hlpf->y_prev = y;
    hlpf->timestamp_prev = timestamp;

    return y;
}
