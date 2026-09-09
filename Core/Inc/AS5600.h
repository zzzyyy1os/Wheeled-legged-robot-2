#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"
#include <math.h>

/* 全局角度变量 (由AS5600任务更新, 其他任务只读) */
extern volatile float as5600_angle;           /* 累积多圈角度 (rad) */
extern volatile float as5600_angle_single;    /* 单圈角度 (rad) */
extern volatile uint16_t as5600_raw;          /* 原始值 (0-4095) */
extern volatile uint8_t  as5600_error;        /* I2C错误计数, 0=正常 */
extern volatile uint8_t  as5600_ready;        /* 传感器就绪标志 */

/* 初始化AS5600 */
void AS5600_Init(void);

/* I2C总线恢复 (SCL时钟拉高释放总线) */
void AS5600_BusRecovery(void);

/* 读取函数 (返回1=成功, 0=失败) */
uint8_t AS5600_Read(void);

/* 兼容旧接口 */
float AS5600_GetAngle_Without_Track(void);
float AS5600_GetAngle(void);

#endif
