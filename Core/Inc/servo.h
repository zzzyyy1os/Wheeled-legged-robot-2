/**
  * @file    servo.h
  * @brief   舵机PWM驱动 (TIM4, PD12~PD15, 300Hz)
  *          脉宽比映射到 0~100
  */
#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

/* 舵机通道号 */
#define SERVO_CH1   0
#define SERVO_CH2   1
#define SERVO_CH3   2
#define SERVO_CH4   3

/**
  * @brief  初始化舵机PWM (TIM4 300Hz, PD12~PD15)
  *         调用后四路PWM输出默认占空比为0
  */
void Servo_Init(void);

/**
  * @brief  设置舵机占空比
  * @param  channel: SERVO_CH1 ~ SERVO_CH4
  * @param  duty: 0 ~ 100 (0=0%, 100=100%)
  */
void Servo_SetDuty(uint8_t channel, float duty);

/**
  * @brief  设置舵机角度 (映射到脉宽)
  * @param  channel: SERVO_CH1 ~ SERVO_CH4
  * @param  angle: 0.0 ~ 100.0 (线性映射到 0~100% 占空比)
  */
void Servo_SetAngle(uint8_t channel, float angle);

/**
  * @brief  使能/失能某路舵机PWM输出
  * @param  channel: SERVO_CH1 ~ SERVO_CH4
  * @param  enable: 1=使能, 0=停止
  */
void Servo_Enable(uint8_t channel, uint8_t enable);

#endif /* __SERVO_H__ */
