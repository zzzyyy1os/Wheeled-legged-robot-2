/**
  * @file    servo.c
  * @brief   舵机PWM驱动 (TIM4, PD12~PD15, 300Hz)
  *
  * 时钟: APB1 Timer = 84MHz
  * 频率: 84000000 / (599+1) / (2799+1) = 50Hz
  * 分辨率: 1/2800 ≈ 0.036%/count
  * 映射: 0~100 → 2.5%~12.5% 占空比 (对应70~350 counts)
  */
#include "servo.h"
#include "tim.h"

#define SERVO_TIM_HANDLE   htim4
#define SERVO_PERIOD       2799

/* 占空比边界 (2.5% 和 12.5% 对应的比较值) */
#define SERVO_MIN_CMP  70   /* 2800 * 2.5%  = 70  */
#define SERVO_MAX_CMP  350  /* 2800 * 12.5% = 350 */

/* 通道号 → HAL通道宏 */
static const uint32_t ch_table[4] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};

/**
  * @brief  初始化舵机PWM
  *         先初始化TIM4 (MX_TIM4_Init), 再启动4路PWM输出
  */
void Servo_Init(void)
{
    MX_TIM4_Init();

    /* 启动4路PWM输出, 初始值为0 (最小占空比) */
    for (uint8_t i = 0; i < 4; i++)
    {
        __HAL_TIM_SET_COMPARE(&SERVO_TIM_HANDLE, ch_table[i], SERVO_MIN_CMP);
        HAL_TIM_PWM_Start(&SERVO_TIM_HANDLE, ch_table[i]);
    }
}

/**
  * @brief  设置舵机脉宽比
  * @param  channel: SERVO_CH1 ~ SERVO_CH4
  * @param  value: 0.0 ~ 100.0 (映射到 2.5%~12.5% 占空比)
  *         0   → 2.5%  (70 counts)
  *         100 → 12.5% (350 counts)
  */
void Servo_SetDuty(uint8_t channel, float value)
{
    if (channel > 3) return;

    /* 钳位 */
    if (value < 0.0f)   value = 0.0f;
    if (value > 100.0f)  value = 100.0f;

    /* 线性映射: 0~100 → SERVO_MIN_CMP~SERVO_MAX_CMP */
    uint32_t compare = SERVO_MIN_CMP
                     + (uint32_t)(value * (SERVO_MAX_CMP - SERVO_MIN_CMP) / 100.0f);

    __HAL_TIM_SET_COMPARE(&SERVO_TIM_HANDLE, ch_table[channel], compare);
}

/**
  * @brief  设置舵机角度
  * @param  channel: SERVO_CH1 ~ SERVO_CH4
  * @param  angle: 0.0 ~ 100.0 (线性映射到 duty 0~100%)
  */
void Servo_SetAngle(uint8_t channel, float angle)
{
    Servo_SetDuty(channel, angle);
}

/**
  * @brief  使能/失能某路舵机PWM输出
  * @param  channel: SERVO_CH1 ~ SERVO_CH4
  * @param  enable: 1=使能, 0=停止输出低电平
  */
void Servo_Enable(uint8_t channel, uint8_t enable)
{
    if (channel > 3) return;

    if (enable)
    {
        HAL_TIM_PWM_Start(&SERVO_TIM_HANDLE, ch_table[channel]);
    }
    else
    {
        HAL_TIM_PWM_Stop(&SERVO_TIM_HANDLE, ch_table[channel]);
    }
}
