/**
  * @file    adc_current.h
  * @brief   ADC电流采样模块 (移植自V3P, 适配STM32F407 HAL)
  *          M1: PA3(IN3)=Ia, PA6(IN6)=Ib
  *          M2: PA4(IN4)=Ia, PA7(IN7)=Ib
  *          支持: 偏移校准, 相电流计算
  */
#ifndef __ADC_CURRENT_H__
#define __ADC_CURRENT_H__

#include "main.h"

#define ADC_CHANNELS    4
#define ADC_VREF        3.3f
#define ADC_RESOLUTION  4096.0f
#define ADC_CONV        (ADC_VREF / ADC_RESOLUTION)  /* 0.00080586V/LSB */

/* 电流传感器参数 (根据实际硬件修改) */
#define SHUNT_RESISTOR  0.01f    /* 采样电阻 0.01Ω */
#define AMP_GAIN        50.0f    /* 运放增益 50倍 */

/* 电流传感器结构体 (兼容V3P Current_Sensor) */
typedef struct {
    int     Sen_Num;       /* 电机编号: 0=M1, 1=M2 */
    float   I_a;           /* A相电流 (安培) */
    float   I_b;           /* B相电流 (安培) */
    float   offset_ia;     /* A相ADC偏移电压 */
    float   offset_ib;     /* B相ADC偏移电压 */
    float   gain_sign;     /* 增益符号: -1.0=V3P取反, +1.0=同相 */
} Current_Sensor_t;

/* ADC DMA缓冲区 */
extern volatile uint16_t adc_dma_buf[ADC_CHANNELS];

/* 初始化ADC + DMA (4通道扫描) */
void ADC_Current_Init(void);

/* 检查ADC是否已启动 */
uint8_t ADC_Is_Started(void);

/* 初始化电流传感器 (校准偏移) */
void CurrSense_Init(Current_Sensor_t *sensor);

/* 获取相电流 (Ia, Ib) */
void GetPhaseCurrent(Current_Sensor_t *sensor);

#endif
