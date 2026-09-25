/**
  * @file    PID.c
  * @brief   PID控制器 (V3P风格, 适配F407)
  *          修复: D项微分作用在测量值上(避免目标变化时脉冲)
  *          修复: 使用DWT微秒定时器(避免HAL_GetTick 1ms分辨率导致D项爆炸)
  *          修复: D项加低通滤波(不完全微分, 抑制高频噪声)
  */
#include "PID.h"

#define LIMIT  6.3f
#define INTEGRATOR_LIMIT 3.0f  /* 积分项限幅 */
#define D_FILTER_ALPHA   0.9f  /* D项低通滤波系数, 越大滤波越强 (0.8~0.95) */

static float _constrain(float amt, float low, float high)
{
    return ((amt < low) ? low : ((amt > high) ? high : amt));
}

/* ======================== DWT微秒定时器 ======================== */

/* 获取当前时间(微秒), 使用DWT周期计数器 */
static inline uint32_t _micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

/* 全局状态 */
static uint32_t Timestamp_Last = 0;
static float Last_Error = 0.0f;
static float Last_intergration = 0.0f;
static float Last_Measurement = 0.0f;
static float filtered_differential = 0.0f;
static uint8_t pid_initialized = 0;

float PID_Controller(float Kp, float Ki, float Kd, float Error, float Measurement)
{
    /* 首次调用, 初始化时间戳 */
    if (!pid_initialized)
    {
        Timestamp_Last = _micros();
        Last_Measurement = Measurement;
        filtered_differential = 0.0f;
        pid_initialized = 1;
    }

    uint32_t now = _micros();
    float Ts = (now - Timestamp_Last) * 1e-6f;  /* us → s */
    Timestamp_Last = now;

    if (Ts <= 0 || Ts > 0.05f) Ts = 0.001f;

    /* P项 */
    float proportion = Kp * Error;

    /* I项: 梯形积分 */
    float intergration = Last_intergration + Ki * 0.5f * Ts * (Error + Last_Error);
    intergration = _constrain(intergration, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);

    /* D项: 微分作用在测量值上, 加低通滤波(不完全微分) */
    float differential_raw = -Kd * (Measurement - Last_Measurement) / Ts;
    filtered_differential = D_FILTER_ALPHA * filtered_differential + (1.0f - D_FILTER_ALPHA) * differential_raw;

    float Output = proportion + intergration + filtered_differential;
    Output = _constrain(Output, -LIMIT, LIMIT);

    Last_Error = Error;
    Last_intergration = intergration;
    Last_Measurement = Measurement;

    return Output;
}

/* ======================== 多实例PID (双电机支持) ======================== */

void PID_Instance_Init(PID_Instance_t *inst)
{
    inst->Timestamp_Last = _micros();
    inst->Last_Error = 0.0f;
    inst->Last_intergration = 0.0f;
    inst->Last_Measurement = 0.0f;
    inst->filtered_differential = 0.0f;
    inst->Integrator_Min = -INTEGRATOR_LIMIT;
    inst->Integrator_Max = INTEGRATOR_LIMIT;
    inst->initialized = 1;
}

void PID_Instance_Reset(PID_Instance_t *inst)
{
    inst->Last_Error = 0.0f;
    inst->Last_intergration = 0.0f;
    inst->Last_Measurement = 0.0f;
    inst->filtered_differential = 0.0f;
    inst->Timestamp_Last = _micros();
}

float PID_Instance_Controller(PID_Instance_t *inst, float Kp, float Ki, float Kd, float Error, float Measurement)
{
    if (!inst->initialized)
    {
        inst->Timestamp_Last = _micros();
        inst->Last_Measurement = Measurement;
        inst->filtered_differential = 0.0f;
        inst->Integrator_Min = -INTEGRATOR_LIMIT;
        inst->Integrator_Max = INTEGRATOR_LIMIT;
        inst->initialized = 1;
    }

    uint32_t now = _micros();
    float Ts = (now - inst->Timestamp_Last) * 1e-6f;  /* us → s */
    inst->Timestamp_Last = now;

    if (Ts <= 0 || Ts > 0.05f) Ts = 0.001f;

    /* P项 */
    float proportion = Kp * Error;

    /* I项: 梯形积分 */
    float intergration = inst->Last_intergration + Ki * 0.5f * Ts * (Error + inst->Last_Error);
    intergration = _constrain(intergration, inst->Integrator_Min, inst->Integrator_Max);

    /* D项: 微分作用在测量值上, 加低通滤波(不完全微分) */
    float differential_raw = -Kd * (Measurement - inst->Last_Measurement) / Ts;
    inst->filtered_differential = D_FILTER_ALPHA * inst->filtered_differential + (1.0f - D_FILTER_ALPHA) * differential_raw;

    float Output = proportion + intergration + inst->filtered_differential;
    Output = _constrain(Output, -LIMIT, LIMIT);

    inst->Last_Error = Error;
    inst->Last_intergration = intergration;
    inst->Last_Measurement = Measurement;

    return Output;
}
