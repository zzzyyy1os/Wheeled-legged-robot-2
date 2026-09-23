/**
  * @file    PID.c
  * @brief   PID控制器 (V3P风格, 适配F407)
  */
#include "PID.h"

#define LIMIT  6.3f
#define INTEGRATOR_LIMIT 3.0f  /* 积分项限幅 */

static float _constrain(float amt, float low, float high)
{
    return ((amt < low) ? low : ((amt > high) ? high : amt));
}

/* 全局状态 */
static uint32_t Timestamp_Last = 0;
static float Last_Error = 0.0f;
static float Last_intergration = 0.0f;
static uint8_t pid_initialized = 0;

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
    intergration = _constrain(intergration, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);  /* 积分限幅 */

    float differential = Kd * (Error - Last_Error) / Ts;

    float Output = proportion + intergration + differential;
    Output = _constrain(Output, -LIMIT, LIMIT);

    Last_Error = Error;
    Last_intergration = intergration;

    return Output;
}

/* ======================== 多实例PID (双电机支持) ======================== */

void PID_Instance_Init(PID_Instance_t *inst)
{
    inst->Timestamp_Last = HAL_GetTick();
    inst->Last_Error = 0.0f;
    inst->Last_intergration = 0.0f;
    inst->Integrator_Min = -INTEGRATOR_LIMIT;
    inst->Integrator_Max = INTEGRATOR_LIMIT;
    inst->initialized = 1;
}

void PID_Instance_Reset(PID_Instance_t *inst)
{
    inst->Last_Error = 0.0f;
    inst->Last_intergration = 0.0f;
    inst->Timestamp_Last = HAL_GetTick();
}

float PID_Instance_Controller(PID_Instance_t *inst, float Kp, float Ki, float Kd, float Error)
{
    if (!inst->initialized)
    {
        inst->Timestamp_Last = HAL_GetTick();
        inst->Integrator_Min = -INTEGRATOR_LIMIT;
        inst->Integrator_Max = INTEGRATOR_LIMIT;
        inst->initialized = 1;
    }

    uint32_t now = HAL_GetTick();
    float Ts = (now - inst->Timestamp_Last) * 1e-3f;
    inst->Timestamp_Last = now;

    if (Ts <= 0 || Ts > 0.05f) Ts = 0.001f;

    float proportion = Kp * Error;

    float intergration = inst->Last_intergration + Ki * 0.5f * Ts * Error;
    intergration = _constrain(intergration, inst->Integrator_Min, inst->Integrator_Max);  /* 积分限幅 */

    float differential = Kd * (Error - inst->Last_Error) / Ts;

    float Output = proportion + intergration + differential;
    Output = _constrain(Output, -LIMIT, LIMIT);

    inst->Last_Error = Error;
    inst->Last_intergration = intergration;

    return Output;
}
