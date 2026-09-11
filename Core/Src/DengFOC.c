/**
  * @file    DengFOC.c
  * @brief   FOC控制库 (V3P风格精简版)
  *          支持: 开环速度/闭环速度/闭环位置
  */
#include "DengFOC.h"
#include "AS5600.h"
#include "PID.h"
#include "LPF.h"
#include "tim.h"

#define PI         3.14159265359f
#define _3PI_2     4.71238898f

/* ======================== 内部变量 ======================== */
float voltage_power_supply = 12.0f;
float shaft_angle = 0.0f;
float open_loop_timestamp = 0.0f;
float zero_electric_angle = 0.0f;

static float Ualpha, Ubeta = 0.0f;
static float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f;
static float dc_a = 0.0f, dc_b = 0.0f, dc_c = 0.0f;

/* ======================== 速度环参数 (外部可调) ======================== */
float vel_Kp           = 0.02f;
float vel_Ki           = 0.05f;
float vel_Kd           = 0.0f;
float vel_actual_speed = 0.0f;
float vel_LPF_Tf       = 0.4f;

/* ======================== 位置环参数 (外部可调) ======================== */
float pos_Kp           = 0.133f;
float pos_Ki           = 0.01f;
float pos_Kd           = 0.0f;
float pos_actual_angle = 0.0f;

/******************************************************************
 * 基础FOC函数
 ******************************************************************/

static float _constrain(float amt, float low, float high)
{
    return ((amt < low) ? low : ((amt > high) ? high : amt));
}

float _normalizeAngle(float angle)
{
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

void setPwm(float Ua, float Ub, float Uc)
{
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (int)(dc_a * htim1.Init.Period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (int)(dc_b * htim1.Init.Period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (int)(dc_c * htim1.Init.Period));
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    Ualpha = -Uq * sin(angle_el);
    Ubeta  =  Uq * cos(angle_el);

    Ua = Ualpha + voltage_power_supply / 2;
    Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPwm(Ua, Ub, Uc);
}

/******************************************************************
 * 传感器电角度
 ******************************************************************/

float getElectricalAngle(void)
{
    return _normalizeAngle((float)(MOTOR_DIR * MOTOR_PP) * as5600_angle_single - zero_electric_angle);
}

/******************************************************************
 * 零电角度校准
 ******************************************************************/

void alignSensor(void)
{
    setPhaseVoltage(3.0f, 0, _3PI_2);
    HAL_Delay(3000);

    zero_electric_angle = getElectricalAngle();

    setPhaseVoltage(0, 0, 0);
    HAL_Delay(500);

    AS5600_Init();
}

/******************************************************************
 * 开环速度控制
 ******************************************************************/

float velocityOpenloop(float target_velocity)
{
    unsigned long now_us = DWT->CYCCNT / (SystemCoreClock / 1000000);

    float Ts = (now_us - open_loop_timestamp) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);

    float Uq = voltage_power_supply / 3;
    setPhaseVoltage(Uq, 0, _normalizeAngle((float)MOTOR_PP * shaft_angle));

    open_loop_timestamp = now_us;
    return Uq;
}

/******************************************************************
 * 闭环速度控制 (V3P风格, 精简)
 ******************************************************************/

void velocityClosedloop_Init(void)
{
    vel_actual_speed = 0.0f;
    /* 重置PID和LPF状态 */
    PID_Init(NULL, 0, 0, 0, 0, 0);
    LPF_Init(NULL, 0);
}

float velocityClosedloop(float target_velocity)
{
    float Vel = Lowpassfilter(vel_LPF_Tf, GetVelocity());
    float Uq  = PID_Controller(vel_Kp, vel_Ki, vel_Kd, MOTOR_DIR * (target_velocity - Vel));
    setPhaseVoltage(Uq, 0, getElectricalAngle());
    vel_actual_speed = Vel;
    return Uq;
}

/******************************************************************
 * 闭环位置控制 (V3P风格, 单环, 精简)
 ******************************************************************/

void positionClosedloop_Init(void)
{
    pos_actual_angle = 0.0f;
}

float positionClosedloop(float target_angle_rad)
{
    float angle = GetAngle();
    float Uq = PID_Controller(pos_Kp, pos_Ki, pos_Kd, (target_angle_rad - MOTOR_DIR * angle) * 180.0f / PI);
    setPhaseVoltage(Uq, 0, getElectricalAngle());
    pos_actual_angle = angle;
    return Uq;
}
