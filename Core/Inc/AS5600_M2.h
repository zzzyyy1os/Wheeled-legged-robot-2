#ifndef __AS5600_M2_H
#define __AS5600_M2_H

#include "main.h"
#include <math.h>

/* M2编码器全局变量 (由AS5600_M2任务更新, 其他任务只读) */
extern volatile float    as5600_m2_angle;           /* 累积多圈角度 (rad) */
extern volatile float    as5600_m2_angle_single;    /* 单圈角度 (rad) */
extern volatile uint16_t as5600_m2_raw;             /* 原始值 (0-4095) */
extern volatile uint8_t  as5600_m2_error;           /* I2C错误计数, 0=正常 */
extern volatile uint8_t  as5600_m2_ready;           /* 传感器就绪标志 */

/* 初始化AS5600 M2 */
void AS5600_M2_Init(void);

/* I2C2总线恢复 */
void AS5600_M2_BusRecovery(void);

/* 读取函数 (返回1=成功, 0=失败) */
uint8_t AS5600_M2_Read(void);

/* V3P风格接口 */
float GetAngle_M2(void);           /* M2多圈角度 */
float GetAngle_NoTrack_M2(void);   /* M2单圈角度 */
float GetVelocity_M2(void);        /* M2速度 (HAL_GetTick计时) */

#endif /* __AS5600_M2_H */
