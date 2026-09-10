/**
  ******************************************************************************
  * @file           : DengFOC.c
  * @brief          : FOC控制库函数 (完整版: 开环/闭环速度/闭环角度)
  *                   移植自DengFOC开源项目, 集成PID和LPF模块
  * @author         : 移植自DengFOC开源项目
  * @version        : 3.0
  * @date           : 2026-08
  * @hardware       : STM32F407VET6 + TIM1三路PWM输出(PE9/PE11/PE13)
  * @motor-pole-pairs : 7（电机极对数，根据实际电机修改）
  * @power-supply   : 12V
  ******************************************************************************
  */

#include "DengFOC.h"
#include "AS5600.h"
#include "encoder.h"
#include "PID.h"
#include "LPF.h"
#include "tim.h"

/* ======================== 内部变量 ======================== */
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float voltage_power_supply = 12.0f;
float shaft_angle = 0.0f;
float open_loop_timestamp = 0.0f;
float zero_electric_angle = 0.0f;

static float Ualpha, Ubeta = 0.0f;
static float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f;
static float dc_a = 0.0f, dc_b = 0.0f, dc_c = 0.0f;

/* ======================== PID/LPF 控制器实例 ======================== */
static PID_HandleTypeDef pid_velocity;
static PID_HandleTypeDef pid_angle;
static LPF_HandleTypeDef lpf_velocity;
static LPF_HandleTypeDef lpf_angle;

/* ======================== 速度环参数 (外部可调) ======================== */
float vel_Kp           = 0.02f;    /* 能自启动不抖动的平衡点 */
float vel_Ki           = 0.05f;    /* 积分帮助克服静摩擦 */
float vel_Kd           = 0.0f;
float vel_Uq_max       = 6.0f;
float vel_integral_max = 6.0f;
float vel_actual_speed = 0.0f;
float vel_LPF_Tf       = 0.05f;    /* 加大滤波, 减少抖动 */

/* ======================== 位置环参数 (外部可调) ======================== */
float pos_Kp           = 20.0f;
float pos_Ki           = 0.0f;
float pos_Kd           = 0.0f;
float pos_Uq_max       = 6.0f;
float pos_actual_angle = 0.0f;
float pos_LPF_Tf       = 0.0f;  /* 0=不滤波 */

/******************************************************************
 * 基础FOC函数
 ******************************************************************/

float _electricalAngle(float shaft_angle, int pole_pairs)
{
    return (shaft_angle * pole_pairs);
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
    angle_el = _normalizeAngle(angle_el);

    /* Park逆变换 */
    Ualpha = -Uq * sin(angle_el);
    Ubeta  =  Uq * cos(angle_el);

    /* Clarke逆变换 */
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
    float sensor_angle = as5600_angle_single;
    return _normalizeAngle((float)(MOTOR_DIR * MOTOR_PP) * sensor_angle - zero_electric_angle);
}

/******************************************************************
 * 零电角度校准
 ******************************************************************/

void alignSensor(void)
{
    setPhaseVoltage(2.0f, 0, _3PI_2);
    HAL_Delay(3000);

    setPhaseVoltage(1.5f, 0, _3PI_2);
    HAL_Delay(500);

    float sensor_angle = as5600_angle_single;
    zero_electric_angle = _normalizeAngle((float)(MOTOR_DIR * MOTOR_PP) * sensor_angle);

    setPhaseVoltage(0, 0, 0);

    /* 重置编码器 */
    AS5600_Init();
    encoder_init();
    encoder_reset();
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
    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, MOTOR_PP));

    open_loop_timestamp = now_us;
    return Uq;
}

/******************************************************************
 * 开环角度控制 (新增, 移植自DengFOC::angleOpenloop)
 ******************************************************************/
float angleOpenloop(float target_angle, float vl)
{
    unsigned long now_us = DWT->CYCCNT / (SystemCoreClock / 1000000);

    float Ts = (now_us - open_loop_timestamp) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    /* P控制器: 角度误差 → 速度 */
    float angle_error = target_angle - shaft_angle;
    float velocity = 20.0f * angle_error;  /* P增益=20 */

    /* 限幅 */
    if (fabs(velocity) > 20.0f)
        velocity = (velocity > 0.0f) ? 20.0f : -20.0f;

    /* 积分角度 */
    shaft_angle = _normalizeAngle(shaft_angle + velocity * Ts);

    /* 电压限幅 */
    float vel = _constrain(vl, 0.0f, voltage_power_supply / 2);
    float Uq = vel * 0.5f;

    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, MOTOR_PP));

    open_loop_timestamp = now_us;
    return Uq;
}

/******************************************************************
 * 闭环速度控制 (PID + LPF, 移植自DengFOC ESP32)
 ******************************************************************/

void velocityClosedloop_Init(void)
{
    /* 初始化PID控制器 */
    PID_Init(&pid_velocity, vel_Kp, vel_Ki, vel_Kd, 1000.0f, vel_Uq_max);

    /* 初始化低通滤波器 */
    LPF_Init(&lpf_velocity, vel_LPF_Tf);

    /* 初始化编码器 */
    encoder_init();
    encoder_reset();

    vel_actual_speed = 0.0f;
}

float velocityClosedloop(float target_velocity)
{
    /* 1. 获取实际速度 (经过低通滤波) */
    float velocity = encoder_get_velocity();
    velocity = LPF_Update(&lpf_velocity, velocity);
    vel_actual_speed = velocity;

    /* 2. PID计算 */
    float Uq = PID_Update(&pid_velocity, target_velocity - velocity);

    /* 3. 获取电角度 */
    float angle_el = getElectricalAngle();

    /* 4. 输出到电机 */
    setPhaseVoltage(Uq, 0, angle_el);

    return Uq;
}

/******************************************************************
 * 闭环位置控制 (PID + LPF, 移植自DengFOC ESP32)
 ******************************************************************/

void positionClosedloop_Init(void)
{
    /* 初始化角度环PID */
    PID_Init(&pid_angle, pos_Kp, pos_Ki, pos_Kd, 1000.0f, 100.0f);

    /* 初始化角度低通滤波器 */
    LPF_Init(&lpf_angle, pos_LPF_Tf);

    /* 初始化速度环PID */
    PID_Init(&pid_velocity, vel_Kp, vel_Ki, vel_Kd, 1000.0f, vel_Uq_max);

    /* 初始化速度低通滤波器 */
    LPF_Init(&lpf_velocity, vel_LPF_Tf);

    pos_actual_angle = 0.0f;
}

float positionClosedloop(float target_angle_rad)
{
    /* 1. 获取当前角度 (经过低通滤波) */
    float angle = encoder_get_angle();
    angle = LPF_Update(&lpf_angle, angle);
    pos_actual_angle = angle;

    /* 2. PID角度环 → 目标速度 */
    float target_velocity = PID_Update(&pid_angle, target_angle_rad - angle);

    /* 3. 获取实际速度 (经过低通滤波) */
    float velocity = encoder_get_velocity();
    velocity = LPF_Update(&lpf_velocity, velocity);
    vel_actual_speed = velocity;

    /* 4. PID速度环 → Uq */
    float Uq = PID_Update(&pid_velocity, target_velocity - velocity);

    /* 5. 获取电角度 */
    float angle_el = getElectricalAngle();

    /* 6. 输出到电机 */
    setPhaseVoltage(Uq, 0, angle_el);

    return Uq;
}
