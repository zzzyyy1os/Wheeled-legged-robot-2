/**
  * @file    PID.h
  * @brief   PID控制器 (V3P风格, 适配F407)
  */
#ifndef __PID_H__
#define __PID_H__

#include "main.h"

/**
  * @brief  PID计算 (全局, M1使用)
  * @param  Measurement: 当前测量值, 用于D项(微分作用在测量值上, 避免目标变化时D项脉冲)
  */
float PID_Controller(float Kp, float Ki, float Kd, float Error, float Measurement);

/* ======================== 多实例PID (双电机支持) ======================== */

typedef struct {
    uint32_t Timestamp_Last;
    float    Last_Error;
    float    Last_intergration;
    float    Last_Measurement;     /* D项微分作用在测量值上 */
    float    filtered_differential; /* D项低通滤波值(不完全微分) */
    uint8_t  initialized;
    float    Integrator_Min;  /* 积分限幅下限 */
    float    Integrator_Max;  /* 积分限幅上限 */
} PID_Instance_t;

void PID_Instance_Init(PID_Instance_t *inst);
void PID_Instance_Reset(PID_Instance_t *inst);
float PID_Instance_Controller(PID_Instance_t *inst, float Kp, float Ki, float Kd, float Error, float Measurement);

#endif /* __PID_H__ */
