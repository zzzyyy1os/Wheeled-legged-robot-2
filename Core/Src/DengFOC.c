/**
  * @file    DengFOC.c
  * @brief   FOC控制库 (V3P风格精简版, 双电机)
  *          M1: TIM1 + AS5600 (I2C3)
  *          M2: TIM2 + AS5600_M2 (I2C2)
  */
#include "DengFOC.h"
#include "AS5600.h"
#include "AS5600_M2.h"
#include "PID.h"
#include "LPF.h"
#include "adc_current.h"
#include "tim.h"

#define PI         3.14159265359f

/* ======================== 内部变量 ======================== */
float voltage_power_supply = 12.0f;
float shaft_angle = 0.0f;
float open_loop_timestamp = 0.0f;
float zero_electric_angle = 0.0f;

static float Ualpha, Ubeta = 0.0f;
static float Ua = 0.0f, Ub = 0.0f, Uc = 0.0f;
static float dc_a = 0.0f, dc_b = 0.0f, dc_c = 0.0f;

/* ======================== 速度环参数 (由freertos.c赋值) ======================== */
float vel_Kp           = 0;
float vel_Ki           = 0;
float vel_Kd           = 0;
float vel_actual_speed = 0;
float vel_LPF_Tf       = 0;

/* ======================== 位置环参数 (由freertos.c赋值) ======================== */
float pos_Kp           = 0;
float pos_Ki           = 0;
float pos_Kd           = 0;
float pos_actual_angle = 0;

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
    HAL_Delay(700);

    zero_electric_angle = getElectricalAngle();

    setPhaseVoltage(0, 0, 0);
    HAL_Delay(100);

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
}

float velocityClosedloop(float target_velocity)
{
    float Vel = Lowpassfilter(vel_LPF_Tf, GetVelocity());
    float Uq  = PID_Controller(vel_Kp, vel_Ki, vel_Kd, MOTOR_DIR * (target_velocity - Vel), MOTOR_DIR * Vel);
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
    float Uq = PID_Controller(pos_Kp, pos_Ki, pos_Kd, (target_angle_rad - MOTOR_DIR * angle) * 180.0f / PI, MOTOR_DIR * angle * 180.0f / PI);
    setPhaseVoltage(Uq, 0, getElectricalAngle());
    pos_actual_angle = angle;
    return Uq;
}

/******************************************************************
 * M2 FOC函数 (TIM2 + I2C2 AS5600)
 ******************************************************************/

/* M2速度环参数 (由freertos.c赋值) */
float vel_m2_Kp           = 0;
float vel_m2_Ki           = 0;
float vel_m2_Kd           = 0;
float vel_m2_actual_speed = 0;
float vel_m2_LPF_Tf       = 0;
float zero_electric_angle_m2 = 0;

/* M2 PID/LPF实例 (独立状态, 不影响M1) */
PID_Instance_t m2_pid_inst;
LPF_Instance_t m2_lpf_inst;

/* M2 PWM设置 */
void setPwm_M2(float Ua, float Ub, float Uc)
{
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (int)(dc_a * htim2.Init.Period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (int)(dc_b * htim2.Init.Period));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (int)(dc_c * htim2.Init.Period));
}

/* M2 SVPWM */
void setPhaseVoltage_M2(float Uq, float Ud, float angle_el)
{
    float Ualpha = -Uq * sin(angle_el);
    float Ubeta  =  Uq * cos(angle_el);

    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;

    setPwm_M2(Ua, Ub, Uc);
}

/* M2电角度 */
float getElectricalAngle_M2(void)
{
    return _normalizeAngle((float)(MOTOR_PP * MOTOR_DIR) * as5600_m2_angle_single - zero_electric_angle_m2);
}

/* M2零电角度校准 */
void alignSensor_M2(void)
{
    setPhaseVoltage_M2(3.0f, 0, _3PI_2);
    HAL_Delay(700);

    zero_electric_angle_m2 = getElectricalAngle_M2();

    setPhaseVoltage_M2(0, 0, 0);
    HAL_Delay(100);

    AS5600_M2_Init();
}

/* M2速度闭环初始化 */
void velocityClosedloop_M2_Init(void)
{
    vel_m2_actual_speed = 0.0f;
    PID_Instance_Init(&m2_pid_inst);

    /* 设置速度环的积分限幅值 */
    m2_pid_inst.Integrator_Min = -2.0f;
    m2_pid_inst.Integrator_Max = 2.0f;

    LPF_Instance_Init(&m2_lpf_inst);
}

/* M2速度闭环控制 */
float velocityClosedloop_M2(float target_velocity)
{
    float Vel = Lowpassfilter_Instance(&m2_lpf_inst, vel_m2_LPF_Tf, GetVelocity_M2());
    float Uq  = PID_Instance_Controller(&m2_pid_inst, vel_m2_Kp, vel_m2_Ki, vel_m2_Kd, MOTOR_DIR * (target_velocity - Vel), MOTOR_DIR * Vel);
    setPhaseVoltage_M2(Uq, 0, getElectricalAngle_M2());
    vel_m2_actual_speed = Vel;
    return Uq;
}

/******************************************************************
 * 电流环 (移植自V3P, 适配F407)
 ******************************************************************/

#define _1_SQRT3  0.57735026919f
#define _2_SQRT3  1.15470053838f

/* M1 电流环参数 (由freertos.c赋值) */
float cur_m1_Kp        = 0;
float cur_m1_Ki        = 0;
float cur_m1_Kd        = 0;
float cur_m1_LPF_Tf    = 0;
float cur_m1_actual_iq = 0;

/* M2 电流环参数 (由freertos.c赋值) */
float cur_m2_Kp        = 0;
float cur_m2_Ki        = 0;
float cur_m2_Kd        = 0;
float cur_m2_LPF_Tf    = 0;
float cur_m2_actual_iq = 0;

/* 电流环 PID/LPF 实例 (独立状态) */
static PID_Instance_t m1_cur_pid_inst;
static LPF_Instance_t m1_cur_lpf_inst;
static PID_Instance_t m2_cur_pid_inst;
static LPF_Instance_t m2_cur_lpf_inst;

/* 电流传感器实例
 * gain_sign: 控制电流反馈极性, +1或-1
 * 如果电流环导致电机来回震荡, 说明极性反了, 需要把gain_sign取反
 * M1和M2的极性可能不同(取决于硬件接线), 需要分别调试
 * 如果电机飞车/震荡, 把gain_sign取反
 */
static Current_Sensor_t current_sensor_m1 = { .Sen_Num = 0, .gain_sign = -1.0f };
static Current_Sensor_t current_sensor_m2 = { .Sen_Num = 1, .gain_sign = 1.0f };

/******************************************************************
 * Clarke + Park 变换: Ia, Ib, θe → Iq
 *   Iα = Ia
 *   Iβ = (1/√3)*Ia + (2/√3)*Ib
 *   Iq = Iβ*cos(θe) - Iα*sin(θe)
 ******************************************************************/
float cal_Iq_Id(float current_a, float current_b, float angle_el)
{
    float I_alpha = current_a;
    float I_beta  = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

    float ct = cosf(angle_el);
    float st = sinf(angle_el);
    float I_q = I_beta * ct - I_alpha * st;

    return I_q;
}

/******************************************************************
 * M1 电流环初始化
 ******************************************************************/
void currentClosedloop_M1_Init(void)
{
    cur_m1_actual_iq = 0.0f;
    PID_Instance_Init(&m1_cur_pid_inst);

    /* 设置电流环的积分限幅值 */
    m1_cur_pid_inst.Integrator_Min = -1.5f;
    m1_cur_pid_inst.Integrator_Max = 1.5f;

    LPF_Instance_Init(&m1_cur_lpf_inst);

    /* 初始化电流传感器 (校准偏移, 电机必须静止!) */
    CurrSense_Init(&current_sensor_m1);
}

/******************************************************************
 * M1 电流环控制
 *   输入: 目标Iq (安培)
 *   输出: Uq电压
 ******************************************************************/
float currentClosedloop_M1(float target_iq)
{
    /* 添加死区处理：小电流时避免振荡 */
    float dead_zone = 0.005f;  // 5mA死区
    if (fabsf(target_iq) < dead_zone)
    {
        target_iq = 0.0f;
    }

    /* 目标为0时重置PID/LPF, 防止累积导致电机不停 */
    if (target_iq == 0.0f)
    {
        PID_Instance_Reset(&m1_cur_pid_inst);
        LPF_Instance_Reset(&m1_cur_lpf_inst);
        setPhaseVoltage(0, 0, getElectricalAngle());
        cur_m1_actual_iq = 0.0f;
        return 0.0f;
    }

    /* 1. 读取相电流 */
    GetPhaseCurrent(&current_sensor_m1);

    /* 2. Clarke+Park变换 → Iq */
    float Iq_raw = cal_Iq_Id(current_sensor_m1.I_a,
                              current_sensor_m1.I_b,
                              getElectricalAngle());

    /* 3. 低通滤波 */
    float Iq_filtered = Lowpassfilter_Instance(&m1_cur_lpf_inst, cur_m1_LPF_Tf, Iq_raw);

    /* 4. 计算误差 - 根据传感器增益符号调整反馈方向 */
    /* 由于gain_sign设置为-1.0f，反馈信号已经取反，所以直接使用 */
    float error = target_iq - Iq_filtered;

    /* 5. PID控制 (D项微分作用在Iq_filtered上) */
    float Uq = PID_Instance_Controller(&m1_cur_pid_inst,
                                        cur_m1_Kp, cur_m1_Ki, cur_m1_Kd,
                                        error, Iq_filtered);

    /* 7. 设置电压 */
    setPhaseVoltage(Uq, 0, getElectricalAngle());

    /* 存储原始反馈值用于显示，与控制逻辑一致 */
    cur_m1_actual_iq = Iq_filtered;
    return Uq;
}

/******************************************************************
 * M2 电流环初始化
 ******************************************************************/
void currentClosedloop_M2_Init(void)
{
    cur_m2_actual_iq = 0.0f;
    PID_Instance_Init(&m2_cur_pid_inst);

    /* 设置电流环的积分限幅值 */
    m2_cur_pid_inst.Integrator_Min = -1.5f;
    m2_cur_pid_inst.Integrator_Max = 1.5f;

    LPF_Instance_Init(&m2_cur_lpf_inst);

    CurrSense_Init(&current_sensor_m2);
}

/******************************************************************
 * M2 电流环控制
 ******************************************************************/
float currentClosedloop_M2(float target_iq)
{
    /* 添加死区处理：小电流时避免振荡 */
    float dead_zone = 0.005f;  // 5mA死区
    if (fabsf(target_iq) < dead_zone)
    {
        target_iq = 0.0f;
    }

    /* 目标为0时重置PID/LPF, 防止累积导致电机不停 */
    if (target_iq == 0.0f)
    {
        PID_Instance_Reset(&m2_cur_pid_inst);
        LPF_Instance_Reset(&m2_cur_lpf_inst);
        setPhaseVoltage_M2(0, 0, getElectricalAngle_M2());
        cur_m2_actual_iq = 0.0f;
        return 0.0f;
    }

    /* 1. 读取相电流 */
    GetPhaseCurrent(&current_sensor_m2);

    /* 2. Clarke+Park变换 → Iq */
    float Iq_raw = cal_Iq_Id(current_sensor_m2.I_a,
                              current_sensor_m2.I_b,
                              getElectricalAngle_M2());

    /* 3. 低通滤波 */
    float Iq_filtered = Lowpassfilter_Instance(&m2_cur_lpf_inst, cur_m2_LPF_Tf, Iq_raw);

    /* 4. 计算误差 - 根据传感器增益符号调整反馈方向 */
    /* 由于gain_sign设置为-1.0f，反馈信号已经取反，所以直接使用 */
    float error = target_iq - Iq_filtered;

    /* 5. PID控制 (D项微分作用在Iq_filtered上) */
    float Uq = PID_Instance_Controller(&m2_cur_pid_inst,
                                        cur_m2_Kp, cur_m2_Ki, cur_m2_Kd,
                                        error, Iq_filtered);

    /* 7. 设置电压 */
    setPhaseVoltage_M2(Uq, 0, getElectricalAngle_M2());

    /* 存储原始反馈值用于显示，与控制逻辑一致 */
    cur_m2_actual_iq = Iq_filtered;
    return Uq;
}

/******************************************************************
 * 速度+电流双闭环 (移植自V3P)
 ******************************************************************/

/* 速度限制 */
float velocity_limit = 10.0f;

/* M1 速度PID/LPF实例 */
static PID_Instance_t m1_vel_pid_inst;
static LPF_Instance_t m1_vel_lpf_inst;

/* M2 速度PID/LPF实例 */
static PID_Instance_t m2_vel_pid_inst;
static LPF_Instance_t m2_vel_lpf_inst;

/******************************************************************
 * M1 速度+电流双闭环初始化
 ******************************************************************/
void velocityCurrentClosedloop_M1_Init(void)
{
    vel_actual_speed = 0.0f;
    PID_Instance_Init(&m1_vel_pid_inst);

    /* 设置M1速度环的积分限幅值 */
    m1_vel_pid_inst.Integrator_Min = -2.0f;
    m1_vel_pid_inst.Integrator_Max = 2.0f;

    LPF_Instance_Init(&m1_vel_lpf_inst);
    currentClosedloop_M1_Init();
}

/******************************************************************
 * M1 速度+电流双闭环
 *   速度PID → 目标电流 → 电流闭环
 ******************************************************************/
float velocityCurrentClosedloop_M1(float target_velocity)
{
    /* 速度限制 */
    target_velocity = _constrain(target_velocity, -velocity_limit, velocity_limit);

    /* 目标为0时重置PID, 防止积分累积导致电机停不下来 */
    if (target_velocity == 0.0f)
    {
        PID_Instance_Reset(&m1_vel_pid_inst);
        LPF_Instance_Reset(&m1_vel_lpf_inst);
        currentClosedloop_M1(0.0f);
        vel_actual_speed = 0.0f;
        return 0.0f;
    }

    /* 速度LPF */
    float Vel = Lowpassfilter_Instance(&m1_vel_lpf_inst, vel_LPF_Tf, GetVelocity());

    /* Anti-windup: 误差大幅变号时重置积分器 (防止急减速时积分累积导致卡顿) */
    float err = MOTOR_DIR * (target_velocity - Vel);
    if (fabsf(err) > 2.0f &&
        ((err > 0 && m1_vel_pid_inst.Last_Error < 0) ||
         (err < 0 && m1_vel_pid_inst.Last_Error > 0)))
    {
        m1_vel_pid_inst.Last_intergration = 0.0f;
    }

    /* 速度PID → 目标电流 (D项微分作用在速度测量值上) */
    float target_iq = PID_Instance_Controller(&m1_vel_pid_inst,
                                               vel_Kp, vel_Ki, vel_Kd,
                                               err,
                                               MOTOR_DIR * Vel);

    /* 电流闭环 */
    currentClosedloop_M1(target_iq);

    vel_actual_speed = Vel;
    return target_iq;
}

/******************************************************************
 * M2 速度+电流双闭环初始化
 ******************************************************************/
void velocityCurrentClosedloop_M2_Init(void)
{
    vel_m2_actual_speed = 0.0f;
    PID_Instance_Init(&m2_vel_pid_inst);

    /* 设置M2速度环的积分限幅值 */
    m2_vel_pid_inst.Integrator_Min = -2.0f;
    m2_vel_pid_inst.Integrator_Max = 2.0f;

    LPF_Instance_Init(&m2_vel_lpf_inst);
    currentClosedloop_M2_Init();
}

/******************************************************************
 * M2 速度+电流双闭环
 ******************************************************************/
float velocityCurrentClosedloop_M2(float target_velocity)
{
    target_velocity = _constrain(target_velocity, -velocity_limit, velocity_limit);

    /* 目标为0时重置PID, 防止积分累积导致电机停不下来 */
    if (target_velocity == 0.0f)
    {
        PID_Instance_Reset(&m2_vel_pid_inst);
        LPF_Instance_Reset(&m2_vel_lpf_inst);
        currentClosedloop_M2(0.0f);
        vel_m2_actual_speed = 0.0f;
        return 0.0f;
    }

    float Vel = Lowpassfilter_Instance(&m2_vel_lpf_inst, vel_m2_LPF_Tf, GetVelocity_M2());

    /* Anti-windup: 误差大幅变号时重置积分器 (防止急减速时积分累积导致卡顿) */
    float err = MOTOR_DIR * (target_velocity - Vel);
    if (fabsf(err) > 2.0f &&
        ((err > 0 && m2_vel_pid_inst.Last_Error < 0) ||
         (err < 0 && m2_vel_pid_inst.Last_Error > 0)))
    {
        m2_vel_pid_inst.Last_intergration = 0.0f;
    }

    float target_iq = PID_Instance_Controller(&m2_vel_pid_inst,
                                               vel_m2_Kp, vel_m2_Ki, vel_m2_Kd,
                                               err,
                                               MOTOR_DIR * Vel);

    currentClosedloop_M2(target_iq);

    vel_m2_actual_speed = Vel;
    return target_iq;
}



