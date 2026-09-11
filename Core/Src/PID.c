/**
  * @file    PID.c
  * @brief   PID控制器 (V3P风格, 适配F407)
  */
#include "PID.h"

#define LIMIT  6.3f

static float _constrain(float amt, float low, float high)
{
    return ((amt < low) ? low : ((amt > high) ? high : amt));
}

/* 全局状态 */
static uint32_t Timestamp_Last = 0;
static float Last_Error = 0.0f;
static float Last_intergration = 0.0f;
static uint8_t pid_initialized = 0;

void PID_Init(PID_HandleTypeDef *hpid, float P, float I, float D, float ramp, float limit)
{
    (void)hpid; (void)P; (void)I; (void)D; (void)ramp; (void)limit;
    Timestamp_Last = HAL_GetTick();
    Last_Error = 0.0f;
    Last_intergration = 0.0f;
    pid_initialized = 1;
}

float PID_Controller(float Kp, float Ki, float Kd, float Error)
{
    /* 首次调用, 初始化时间戳 */
    if (!pid_initialized)
    {
        Timestamp_Last = HAL_GetTick();
        pid_initialized = 1;
    }

    uint32_t now = HAL_GetTick();
    float Ts = (now - Timestamp_Last) * 1e-3f;  /* ms → s */
    Timestamp_Last = now;

    if (Ts <= 0 || Ts > 0.05f) Ts = 0.001f;

    float proportion = Kp * Error;

    float intergration = Last_intergration + Ki * 0.5f * Ts * Error;
    intergration = _constrain(intergration, -LIMIT, LIMIT);

    float differential = Kd * (Error - Last_Error) / Ts;

    float Output = proportion + intergration + differential;
    Output = _constrain(Output, -LIMIT, LIMIT);

    Last_Error = Error;
    Last_intergration = intergration;

    return Output;
}

/* 保留旧接口兼容 */
float PID_Update(PID_HandleTypeDef *hpid, float error)
{
    (void)hpid;
    return PID_Controller(0, 0, 0, error);
}
