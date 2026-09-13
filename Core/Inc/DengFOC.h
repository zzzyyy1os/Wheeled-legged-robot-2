/**
  * @file    DengFOC.h
  * @brief   FOC控制库 (V3P风格精简版, 双电机)
  *          M1: TIM1 PWM + I2C3 AS5600
  *          M2: TIM2 PWM + I2C2 AS5600
  */
#ifndef __DENGFOC_H__
#define __DENGFOC_H__

#include "main.h"
#include "PID.h"
#include "LPF.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265359f
#endif

#define _3PI_2 4.71238898038f

/* 电机参数 (根据实际电机修改) */
#define MOTOR_PP   7     /* 极对数 */
#define MOTOR_DIR  (1)   /* 传感器方向, 1或-1 */

/* ======================== M1 FOC函数 (TIM1) ======================== */
float _normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
float getElectricalAngle(void);
void alignSensor(void);
float velocityOpenloop(float target_velocity);
void  velocityClosedloop_Init(void);
float velocityClosedloop(float target_velocity);
void  positionClosedloop_Init(void);
float positionClosedloop(float target_angle_rad);

/* M1速度环参数 (可调) */
extern float vel_Kp;
extern float vel_Ki;
extern float vel_Kd;
extern float vel_actual_speed;
extern float vel_LPF_Tf;

/* M1位置环参数 (可调) */
extern float pos_Kp;
extern float pos_Ki;
extern float pos_Kd;
extern float pos_actual_angle;

extern float zero_electric_angle;

/* ======================== M2 FOC函数 (TIM2) ======================== */
void  setPwm_M2(float Ua, float Ub, float Uc);
void  setPhaseVoltage_M2(float Uq, float Ud, float angle_el);
float getElectricalAngle_M2(void);
void  alignSensor_M2(void);
void  velocityClosedloop_M2_Init(void);
float velocityClosedloop_M2(float target_velocity);

/* M2速度环参数 (可调) */
extern float vel_m2_Kp;
extern float vel_m2_Ki;
extern float vel_m2_Kd;
extern float vel_m2_actual_speed;
extern float vel_m2_LPF_Tf;
extern float zero_electric_angle_m2;

/* M2 PID/LPF实例 (独立状态) */
extern PID_Instance_t m2_pid_inst;
extern LPF_Instance_t m2_lpf_inst;

#endif /* __DENGFOC_H__ */
