#ifndef __DENGFOC_H__
#define __DENGFOC_H__

#include "main.h"
#include <math.h>

#ifndef PI
#define PI M_PI
#endif

#define _3PI_2 4.71238898038f

/* 电机参数 (根据实际电机修改) */
#define MOTOR_PP   7     /* 极对数 */
#define MOTOR_DIR  (1)   /* 传感器方向, 1或-1 */

/* 基础FOC函数 */
float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
void setPwm(float Ua, float Ub, float Uc);
void setPhaseVoltage(float Uq, float Ud, float angle_el);

/* 传感器电角度 (从AS5600读取, 含DIR和零偏校正) */
float getElectricalAngle(void);

/* 零电角度校准 (上电时调用一次, 电机对齐) */
void alignSensor(void);

/* 开环速度控制 */
float velocityOpenloop(float target_velocity);

/* 闭环速度控制 (PI控制器) */
void  velocityClosedloop_Init(void);
float velocityClosedloop(float target_velocity, float Ts);

/* 闭环位置控制 (P控制器) */
void  positionClosedloop_Init(void);
float positionClosedloop(float target_angle_rad);

/* 速度环参数 */
extern float vel_Kp;
extern float vel_Ki;
extern float vel_Uq_max;
extern float vel_integral_max;
extern float vel_actual_speed;

/* 位置环参数 */
extern float pos_Kp;          /* 位置环比例增益 */
extern float pos_Uq_max;      /* 位置环Uq输出限幅 */
extern float pos_actual_angle; /* 当前实际角度 rad (供外部读取) */

/* 零电角度偏移 (校准后自动设置) */
extern float zero_electric_angle;

#endif
