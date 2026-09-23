#include "key.h"
#include "FreeRTOS.h"
#include "task.h"

/* 按键实例数组 */
static Key_t keys[KEY_NUM] = {
    { .port = GPIOA, .pin = GPIO_PIN_0 },  /* KEY_1: PA0 */
    { .port = GPIOB, .pin = GPIO_PIN_0 },  /* KEY_2: PB0 */
    { .port = GPIOB, .pin = GPIO_PIN_1 },  /* KEY_3: PB1 */
};

/* 按键事件队列 (每个按键一个事件) */
static volatile KeyEvent_t key_events[KEY_NUM] = { KEY_EVENT_NONE };

/* 初始化按键 */
void Key_Init(void)
{
    for (int i = 0; i < KEY_NUM; i++)
    {
        keys[i].state = 0;
        keys[i].last_raw = 0;
        keys[i].debounce_tick = 0;
        keys[i].press_tick = 0;
        keys[i].long_triggered = 0;
        key_events[i] = KEY_EVENT_NONE;
    }
}

/* 按键扫描 (周期性调用, 建议 10ms) */
void Key_Scan(void)
{
    uint32_t now = xTaskGetTickCount();

    for (int i = 0; i < KEY_NUM; i++)
    {
        /* 读取引脚电平 (高电平有效 = 按下, 按键接3.3V) */
        uint8_t raw = (HAL_GPIO_ReadPin(keys[i].port, keys[i].pin) == GPIO_PIN_SET) ? 1 : 0;

        /* 消抖处理 */
        if (raw != keys[i].last_raw)
        {
            keys[i].last_raw = raw;
            keys[i].debounce_tick = now;
        }

        /* 消抖时间到, 更新稳定状态 */
        if ((now - keys[i].debounce_tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS))
        {
            if (raw != keys[i].state)
            {
                keys[i].state = raw;

                if (raw)  /* 高电平 = 按下 */
                {
                    /* 按下事件 */
                    key_events[i] = KEY_EVENT_PRESS;
                    keys[i].press_tick = now;
                    keys[i].long_triggered = 0;
                }
                else  /* 低电平 = 释放 */
                {
                    /* 释放事件 (如果长按未触发, 才触发释放) */
                    if (!keys[i].long_triggered)
                    {
                        key_events[i] = KEY_EVENT_RELEASE;
                    }
                }
            }

            /* 长按检测 */
            if (keys[i].state && !keys[i].long_triggered)
            {
                if ((now - keys[i].press_tick) >= pdMS_TO_TICKS(KEY_LONG_PRESS_MS))
                {
                    key_events[i] = KEY_EVENT_LONG_PRESS;
                    keys[i].long_triggered = 1;
                }
            }
        }
    }
}

/* 获取按键事件 (读取后自动清除) */
KeyEvent_t Key_GetEvent(KeyId_t id)
{
    if (id >= KEY_NUM) return KEY_EVENT_NONE;
    KeyEvent_t event = key_events[id];
    key_events[id] = KEY_EVENT_NONE;
    return event;
}

/* 获取按键当前状态 */
uint8_t Key_IsPressed(KeyId_t id)
{
    if (id >= KEY_NUM) return 0;
    return keys[id].state;
}
