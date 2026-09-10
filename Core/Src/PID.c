/**
  * @file    PID.c
  * @brief   PID 控制器 (移植自 DengFOC, 适配 STM32 HAL)
  */
#include "PID.h"

void PID_Init(PID_HandleTypeDef *hpid, float P, float I, float D, float ramp, float limit)
{
    hpid->P = P;
    hpid->I = I;
    hpid->D = D;
    hpid->output_ramp = ramp;
    hpid->limit = limit;

    hpid->error_prev = 0.0f;
    hpid->output_prev = 0.0f;
    hpid->integral_prev = 0.0f;
    hpid->timestamp_prev = HAL_GetTick();
}

float PID_Update(PID_HandleTypeDef *hpid, float error)
{
    uint32_t timestamp_now = HAL_GetTick();
    float Ts = (timestamp_now - hpid->timestamp_prev) * 1e-3f;

    /* 时间异常处理 */
    if (Ts <= 0.0f || Ts > 0.5f)
    {
        Ts = 1e-3f;
    }

    hpid->timestamp_prev = timestamp_now;

    /* P 项 */
    float proportional = hpid->P * error;

    /* I 项 (梯形积分) */
    hpid->integral_prev += hpid->I * Ts * 0.5f * (error + hpid->error_prev);

    /* 积分限幅 */
    if (hpid->integral_prev > hpid->limit)
        hpid->integral_prev = hpid->limit;
    else if (hpid->integral_prev < -hpid->limit)
        hpid->integral_prev = -hpid->limit;

    /* D 项 (一阶差分) */
    float derivative = hpid->D * (error - hpid->error_prev) / Ts;

    /* 计算输出 */
    float output = proportional + hpid->integral_prev + derivative;

    /* 输出限幅 */
    if (output > hpid->limit)
        output = hpid->limit;
    else if (output < -hpid->limit)
        output = -hpid->limit;

    /* 输出变化率限幅 (ramp) */
    if (hpid->output_ramp > 0.0f)
    {
        float delta = output - hpid->output_prev;
        if (delta > hpid->output_ramp * Ts)
            output = hpid->output_prev + hpid->output_ramp * Ts;
        else if (delta < -hpid->output_ramp * Ts)
            output = hpid->output_prev - hpid->output_ramp * Ts;
    }

    /* 保存状态 */
    hpid->error_prev = error;
    hpid->output_prev = output;

    return output;
}
