/**
  ******************************************************************************
  * @file           : DengFOC.c
  * @brief          : FOC控制库函数
  *                   包含电角度计算、归一化、PWM输出设置、
  *                   相电压设置、开环/闭环速度控制等核心函数。
  * @author         : 移植自DengFOC开源项目
  * @version        : 2.0
  * @date           : 2026-08
  * @hardware       : STM32F407VET6 + TIM1三路PWM输出(PE9/PE11/PE13)
  * @motor-pole-pairs : 7（电机极对数，根据实际电机修改）
  * @power-supply   : 12V
  ******************************************************************************
  * @attention
  *
  * 本代码基于DengFOC开源项目移植，用于STM32F407平台的FOC速度控制。
  * 支持开环控制(velocityOpenloop)和闭环PI控制(velocityClosedloop)。
  * 闭环模式通过AS5600磁编码器读取实际角度，PI控制器调节Uq输出。
  *
  ******************************************************************************
  */

#include "DengFOC.h"
#include "AS5600.h"
#include "tim.h"

//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))//宏定义实现的一个约束函数,用于限制一个值的范围。
float voltage_power_supply=12;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;

/******************************************************************
 * 函 数 名 称：_electricalAngle
 * 函 数 说 明：电角度求解
 * 函 数 形 参：shaft_angle(机械角度, 单位rad), pole_pairs(电机极对数)
 * 函 数 返 回：float 电角度(机械角度×极对数)
 * 备       注：无
******************************************************************/
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

/******************************************************************
 * 函 数 名 称：_normalizeAngle
 * 函 数 说 明：电角度归一化
 * 函 数 形 参：angle(待归一化的角度, 单位rad)
 * 函 数 返 回：float 归一化后的角度[0, 2π]
 * 备       注：无
******************************************************************/
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}

/******************************************************************
 * 函 数 名 称：setPwm
 * 函 数 说 明：设置PWM输出
 * 函 数 形 参：Ua(A相电压), Ub(B相电压), Uc(C相电压)
 * 函 数 返 回：void 无返回值
 * 备       注：无
******************************************************************/
void setPwm(float Ua, float Ub, float Uc) {

  // 计算占空比
  // 限制占空比从0到1
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //写入PWM到TIM1 CH1/CH2/CH3
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (int)(dc_a * htim1.Init.Period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (int)(dc_b * htim1.Init.Period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (int)(dc_c * htim1.Init.Period));
}

/******************************************************************
 * 函 数 名 称：setPhaseVoltage
 * 函 数 说 明：设置相电压
 * 函 数 形 参：Uq(q轴电压), Ud(d轴电压), angle_el(电角度, 单位rad)
 * 函 数 返 回：void 无返回值
 * 备       注：无
******************************************************************/
void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}

/******************************************************************
 * 函 数 名 称：velocityOpenloop
 * 函 数 说 明：开环速度控制
 * 函 数 形 参：target_velocity(目标角速度, 单位rad/s)
 * 函 数 返 回：float Uq(q轴电压值)
 * 备       注：无
******************************************************************/
float velocityOpenloop(float target_velocity){
  unsigned long now_us = DWT->CYCCNT / (SystemCoreClock / 1000000);  //通过DWT周期计数器获取微秒级时间戳
  
  //计算当前每个Loop的运行时间间隔
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

  //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
  

  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

  // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
  // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
  float Uq = voltage_power_supply/3;
  
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, 7));
  
  open_loop_timestamp = now_us;  //用于计算下一个时间间隔

  return Uq;
}

/******************************************************************
 * 零电角度校准 & 传感器电角度
 ******************************************************************/

/******************************************************************
 * 函 数 名 称：alignSensor
 * 函 数 说 明：零电角度校准
 *               给定 _3PI_2 方向电压, 等待电机对齐,
 *               读取此时的电角度作为 zero_electric_angle。
 * 函 数 形 参：无
 * 函 数 返 回：void
 * 备       注：上电时调用一次, 会阻塞约2秒
******************************************************************/
void alignSensor(void)
{
  /* 给 _3PI_2 方向2V电压, 使电机转子对齐到270°电角度位置 */
  setPhaseVoltage(2.0f, 0, _3PI_2);
  HAL_Delay(3000);  /* 等待电机稳定对齐 */

  /* 额外等待500ms, 确保电机完全静止 */
  setPhaseVoltage(1.5f, 0, _3PI_2);
  HAL_Delay(500);

  /* 读取全局角度变量 (由AS5600任务更新) */
  float sensor_angle = as5600_angle_single;
  zero_electric_angle = _normalizeAngle((float)(MOTOR_DIR * MOTOR_PP) * sensor_angle);

  /* 释放电压 */
  setPhaseVoltage(0, 0, 0);

  /* 重置AS5600累积角度跟踪, 让位置反馈从对齐位置开始 */
  AS5600_Init();
}

/******************************************************************
 * 函 数 名 称：getElectricalAngle
 * 函 数 说 明：从AS5600读取传感器角度, 计算当前电角度
 *               电角度 = DIR × PP × 单圈机械角度
 *               (zero_electric_angle 在 setPhaseVoltage 中补偿)
 * 函 数 形 参：无
 * 函 数 返 回：float 电角度 (rad)
 * 备       注：用于闭环控制中替代 _electricalAngle(shaft_angle, 7)
******************************************************************/
float getElectricalAngle(void)
{
  /* 与Arduino参考一致: DIR×PP×sensor - zero_electric_angle */
  float sensor_angle = as5600_angle_single;
  return _normalizeAngle((float)(MOTOR_DIR * MOTOR_PP) * sensor_angle - zero_electric_angle);
}

/******************************************************************
 * 闭环速度控制相关变量
 ******************************************************************/
/* PI控制器参数 (默认值, 可通过UART在线调整) */
float vel_Kp           = 0.5f;    /* 比例增益 */
float vel_Ki           = 0.1f;    /* 积分增益 */
float vel_Uq_max       = 6.0f;    /* Uq最大输出 (voltage_power_supply/2) */
float vel_integral_max = 3.0f;    /* 积分项限幅 */
float vel_actual_speed = 0.0f;    /* 当前实际速度 (外部可读) */

/* 内部状态 */
static float vel_integral    = 0.0f;
static float vel_angle_prev  = 0.0f;
static uint32_t vel_ts_prev  = 0;
static uint8_t vel_first_run = 1;   /* 首次运行标志 */

/******************************************************************
 * 函 数 名 称：velocityClosedloop_Init
 * 函 数 说 明：闭环速度控制初始化 (在启动电机前调用一次)
 * 函 数 形 参：无
 * 函 数 返 回：void
 * 备       注：复位PI积分项, 读取当前传感器角度作为初始值
******************************************************************/
void velocityClosedloop_Init(void)
{
  vel_integral   = 0.0f;
  vel_actual_speed = 0.0f;
  vel_ts_prev    = 0;
  vel_first_run  = 1;
  /* 读取当前实际角度, 避免第一帧算出虚假速度 */
  vel_angle_prev = as5600_angle;
}

/******************************************************************
 * 函 数 名 称：velocityClosedloop
 * 函 数 说 明：闭环速度控制 (PI控制器)
 *               通过AS5600读取实际机械角度, 用DWT实测采样时间,
 *               PI控制器输出Uq, 实际电角度驱动FOC。
 * 函 数 形 参：target_velocity (目标角速度, rad/s)
 *              Ts (未使用, 保留接口兼容; 实际使用DWT实测)
 * 函 数 返 回：float Uq (实际输出的q轴电压)
 * 备       注：需在for循环中调用, 建议1ms周期
******************************************************************/
float velocityClosedloop(float target_velocity, float Ts)
{
  /* ---- 1. 获取实际机械角度 (rad, 累积多圈, 从全局变量读取) ---- */
  float sensor_angle = as5600_angle;

  /* ---- 2. 用DWT实测采样时间 ---- */
  uint32_t now_us = DWT->CYCCNT / (SystemCoreClock / 1000000);

  if (vel_first_run) {
    /* 第一帧: 只记录时间戳, 不输出电压, 避免虚假速度跳变 */
    vel_ts_prev   = now_us;
    vel_angle_prev = sensor_angle;
    vel_first_run  = 0;
    return 0.0f;
  }

  float dt = (now_us - vel_ts_prev) * 1e-6f;
  vel_ts_prev = now_us;
  if (dt <= 0 || dt > 0.5f) dt = 1e-3f;

  /* ---- 3. 计算实际速度 (rad/s) ---- */
  float d_angle = sensor_angle - vel_angle_prev;
  vel_angle_prev = sensor_angle;
  vel_actual_speed = d_angle / dt;   /* 机械角速度 */

  /* ---- 4. PI控制器 ---- */
  float error = target_velocity - vel_actual_speed;

  vel_integral += vel_Ki * error * dt;
  vel_integral = _constrain(vel_integral, -vel_integral_max, vel_integral_max);

  float Uq = vel_Kp * error + vel_integral;
  Uq = _constrain(Uq, -vel_Uq_max, vel_Uq_max);

  /* ---- 5. 用电角度驱动FOC ---- */
  float angle_el = getElectricalAngle();
  setPhaseVoltage(Uq, 0, angle_el);

  return Uq;
}

/******************************************************************
 * 闭环位置控制相关变量
 ******************************************************************/
float pos_Kp           = 0.05f;   /* 位置环比例增益 */
float pos_Uq_max       = 6.0f;    /* 位置环Uq输出限幅 */
float pos_actual_angle = 0.0f;    /* 当前实际角度 rad */

/******************************************************************
 * 函 数 名 称：positionClosedloop_Init
 * 函 数 说 明：闭环位置控制初始化
 * 函 数 形 参：无
 * 函 数 返 回：void
******************************************************************/
void positionClosedloop_Init(void)
{
  pos_actual_angle = 0.0f;
}

/******************************************************************
 * 函 数 名 称：positionClosedloop
 * 函 数 说 明：闭环位置控制 (P控制器)
 *               参考Arduino FOC代码:
 *               Uq = Kp × (目标角度 - DIR×实际角度) × 180/PI
 *               直接输出到 setPhaseVoltage, 使用传感器电角度。
 * 函 数 形 参：target_angle_rad (目标角度, 单位rad, 累积多圈)
 * 函 数 返 回：float Uq (实际输出的q轴电压)
 * 备       注：需在for循环中调用, 建议1ms周期
******************************************************************/
float positionClosedloop(float target_angle_rad)
{
  /* ---- 1. 读取实际角度 (累积多圈, 从全局变量读取) ---- */
  float sensor_angle = as5600_angle;
  pos_actual_angle = sensor_angle;

  /* ---- 2. 位置误差 (rad → 度) ---- */
  /* 注意: DIR 只影响电角度, 不影响位置反馈 */
  float error_deg = (target_angle_rad - sensor_angle) * 180.0f / PI;

  /* ---- 3. P控制器输出Uq ---- */
  float Uq = pos_Kp * error_deg;
  Uq = _constrain(Uq, -pos_Uq_max, pos_Uq_max);

  /* ---- 4. 用电角度驱动FOC ---- */
  float angle_el = getElectricalAngle();
  setPhaseVoltage(Uq, 0, angle_el);

  return Uq;
}