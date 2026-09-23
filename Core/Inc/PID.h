/**
  * @file    PID.h
  * @brief   PID控制器 (V3P风格, 适配F407)
  */
#ifndef __PID_H__
#define __PID_H__

#include "main.h"

/**
  * @brief  PID计算 (全局, M1使用)
  */
float PID_Controller(float Kp, float Ki, float Kd, float Error);

/* ======================== 多实例PID (双电机支持) ======================== */

typedef struct {
    uint32_t Timestamp_Last;
    float    Last_Error;
    float    Last_intergration;
    uint8_t  initialized;
    float    Integrator_Min;  /* 积分限幅下限 */
    float    Integrator_Max;  /* 积分限幅上限 */
} PID_Instance_t;

void PID_Instance_Init(PID_Instance_t *inst);
void PID_Instance_Reset(PID_Instance_t *inst);
float PID_Instance_Controller(PID_Instance_t *inst, float Kp, float Ki, float Kd, float Error);

#endif /* __PID_H__ */
