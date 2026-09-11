/**
  * @file    DengFOC.h
  * @brief   FOC控制库 (V3P风格精简版)
  *          支持: 开环速度/闭环速度/闭环位置
  */
#ifndef __DENGFOC_H__
#define __DENGFOC_H__

#include "main.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265359f
#endif

#define _3PI_2 4.71238898038f

/* 电机参数 (根据实际电机修改) */
#define MOTOR_PP   7     /* 极对数 */
#define MOTOR_DIR  (1)   /* 传感器方向, 1或-1 */

/* 基础FOC函数 */
float _normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);

/* 传感器电角度 (从AS5600读取, 含DIR和零偏校正) */
float getElectricalAngle(void);

/* 零电角度校准 (上电时调用一次, 电机对齐) */
void alignSensor(void);

/* 开环速度控制 */
float velocityOpenloop(float target_velocity);

/* 闭环速度控制 (V3P风格) */
void  velocityClosedloop_Init(void);
float velocityClosedloop(float target_velocity);

/* 闭环位置控制 (V3P风格, 单环) */
void  positionClosedloop_Init(void);
float positionClosedloop(float target_angle_rad);

/* 速度环参数 (可调) */
extern float vel_Kp;
extern float vel_Ki;
extern float vel_Kd;
extern float vel_actual_speed;
extern float vel_LPF_Tf;

/* 位置环参数 (可调) */
extern float pos_Kp;
extern float pos_Ki;
extern float pos_Kd;
extern float pos_actual_angle;

/* 零电角度偏移 (校准后自动设置) */
extern float zero_electric_angle;

#endif /* __DENGFOC_H__ */
