#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/* 按键数量 */
#define KEY_NUM          3

/* 按键消抖时间 (ms) */
#define KEY_DEBOUNCE_MS  20

/* 长按时间 (ms) */
#define KEY_LONG_PRESS_MS 1000

/* 按键编号定义 */
typedef enum {
    KEY_1 = 0,   /* PA0 */
    KEY_2 = 1,   /* PB0 */
    KEY_3 = 2,   /* PB1 */
} KeyId_t;

/* 按键事件类型 */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESS,       /* 按下 */
    KEY_EVENT_RELEASE,     /* 释放 */
    KEY_EVENT_LONG_PRESS,  /* 长按 */
} KeyEvent_t;

/* 按键状态结构体 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       state;          /* 当前稳定状态 (0=释放, 1=按下) 高电平有效 */
    uint8_t       last_raw;       /* 上次原始电平 (高电平有效) */
    uint32_t      debounce_tick;  /* 消抖计时 */
    uint32_t      press_tick;     /* 按下时刻 (用于长按检测) */
    uint8_t       long_triggered; /* 长按已触发标志 */
} Key_t;

/* 初始化按键GPIO和状态 */
void Key_Init(void);

/* 周期性调用 (在任务中调用, 建议 10ms 周期) */
void Key_Scan(void);

/* 获取按键事件 (返回后自动清除) */
KeyEvent_t Key_GetEvent(KeyId_t id);

/* 获取按键当前状态 (0=释放, 1=按下) */
uint8_t Key_IsPressed(KeyId_t id);

#endif /* __KEY_H */
