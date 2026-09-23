/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : 双电机FOC速度闭环控制
  *                      - AS5600Task:    M1编码器读取 (I2C3)
  *                      - AS5600M2Task:  M2编码器读取 (I2C2)
  *                      - MotorTask:     M1+M2速度闭环控制
  *                      - OLEDTask:      分屏显示双电机参数
  *                      - UARTTask:      串口DMA接收A/B命令
  *                      - KeyTask:       按键检测 (PA0/PB0/PB1)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "DengFOC.h"
#include "AS5600.h"
#include "AS5600_M2.h"
#include "uart_comm.h"
#include "OLED.h"
#include "adc_current.h"
#include "key.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* =========================================================================
 * 用户配置区 - 双电机PID参数
 * ========================================================================= */

/* M1 速度环 PID 参数 */
#define M1_VEL_KP        0.1f
#define M1_VEL_KI        0.0f
#define M1_VEL_KD        0.0f
#define M1_VEL_LPF_TF    0.5f

/* M2 速度环 PID 参数 */
#define M2_VEL_KP        0.1f
#define M2_VEL_KI        0.0f
#define M2_VEL_KD        0.0f
#define M2_VEL_LPF_TF    0.5f

/* M1 电流环 PID 参数 */
#define M1_CUR_KP        6.0f
#define M1_CUR_KI        0.0f
#define M1_CUR_KD        0.0f
#define M1_CUR_LPF_TF    0.02f

/* M2 电流环 PID 参数 */
#define M2_CUR_KP        6.0f
#define M2_CUR_KI        0.0f
#define M2_CUR_KD        0.0f
#define M2_CUR_LPF_TF    0.02f

/* 速度限制 (rad/s) */
#define VELOCITY_LIMIT   30.0f

/* 控制模式选择: 0=速度环, 1=电流环, 2=ADC诊断, 3=速度+电流双闭环, 4=三环嵌套 */
#define CURRENT_LOOP_TEST  1

/* ========================================================================= */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static volatile float m1_target_velocity = 0.0f;
static volatile float m2_target_velocity = 0.0f;

/* 电流环目标 (安培) */
static volatile float m1_target_current = 0.0f;   /* 默认0A, 通过串口C/D命令设置 */
static volatile float m2_target_current = 0.0f;

/* OLED翻页 (0=电机状态, 1=PID参数) */
static volatile uint8_t oled_page = 0;

/* 系统就绪标志 */
static volatile uint8_t system_ready = 0;

/* USER CODE END Variables */

/* Definitions for AS5600Task (M1) */
osThreadId_t AS5600TaskHandle;
const osThreadAttr_t AS5600Task_attributes = {
  .name = "AS5600Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for AS5600M2Task (M2) */
osThreadId_t AS5600M2TaskHandle;
const osThreadAttr_t AS5600M2Task_attributes = {
  .name = "AS5600M2Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* ADC测试任务 */
osThreadId_t ADCTestTaskHandle;
const osThreadAttr_t ADCTestTask_attributes = {
  .name = "ADCTestTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* 按键检测任务 */
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Function prototypes */
void StartAS5600Task(void *argument);
void StartAS5600M2Task(void *argument);
void StartMotorTask(void *argument);
void StartOLEDTask(void *argument);
void StartUARTTask(void *argument);
void StartADCTestTask(void *argument);
void StartKeyTask(void *argument);

/* USER CODE BEGIN Init */
/* USER CODE END Init */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  AS5600TaskHandle   = osThreadNew(StartAS5600Task,   NULL, &AS5600Task_attributes);
  AS5600M2TaskHandle = osThreadNew(StartAS5600M2Task, NULL, &AS5600M2Task_attributes);
  MotorTaskHandle    = osThreadNew(StartMotorTask,    NULL, &MotorTask_attributes);
  UARTTaskHandle     = osThreadNew(StartUARTTask,     NULL, &UARTTask_attributes);
  OLEDTaskHandle     = osThreadNew(StartOLEDTask,     NULL, &OLEDTask_attributes);
  ADCTestTaskHandle  = osThreadNew(StartADCTestTask,  NULL, &ADCTestTask_attributes);
  KeyTaskHandle      = osThreadNew(StartKeyTask,       NULL, &KeyTask_attributes);

  UART_Comm_Init();

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
}

/*============================================================================
 * AS5600Task - M1编码器读取 (500Hz, I2C3)
 *============================================================================*/
void StartAS5600Task(void *argument)
{
    AS5600_Init();

    for (;;)
    {
        if (!AS5600_Read())
        {
            static uint8_t fail_cnt = 0;
            if (++fail_cnt >= 20)
            {
                static uint8_t print_flag = 0;
                if (!print_flag) {
                    print_flag = 1;
                }
                fail_cnt = 0;
            }
        }
        osDelay(2);
    }
}

/*============================================================================
 * AS5600M2Task - M2编码器读取 (500Hz, I2C2)
 *============================================================================*/
void StartAS5600M2Task(void *argument)
{
    AS5600_M2_Init();

    for (;;)
    {
        if (!AS5600_M2_Read())
        {
            static uint8_t fail_cnt = 0;
            if (++fail_cnt >= 20)
            {
                static uint8_t print_flag = 0;
                if (!print_flag) {
                    print_flag = 1;
                }
                fail_cnt = 0;
            }
        }
        osDelay(2);
    }
}

/*============================================================================
 * MotorTask - 双电机闭环控制 (1kHz)
 *   CURRENT_LOOP_TEST=0: 速度闭环模式
 *   CURRENT_LOOP_TEST=1: 电流环测试模式 (移植自V3P)
 *============================================================================*/
void StartMotorTask(void *argument)
{
    /* ---- M1 硬件初始化 ---- */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* ---- M2 硬件初始化 ---- */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    /* DWT微秒定时器 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* ---- 等待M1编码器就绪 ---- */
    while (!as5600_ready) { osDelay(10); }

    /* ---- 等待M2编码器就绪 ---- */
    while (!as5600_m2_ready) { osDelay(10); }

    /* ---- M1 零电角度校准 ---- */
    alignSensor();

    /* ---- M2 零电角度校准 ---- */
    alignSensor_M2();

    /* ---- 关键: 对齐后停止所有PWM输出, 等待电流降为0 ---- */
    setPhaseVoltage(0, 0, 0);
    setPhaseVoltage_M2(0, 0, 0);
    osDelay(1000);  /* 等待1秒让电机完全静止 */

    /* 设置系统就绪标志并发送串口消息 */
    system_ready = 1;
    UART_SendString("ALL OK\r\n");

    /* 等待3秒后再发送一次ALL OK */
    osDelay(3000);
    UART_SendString("ALL OK\r\n");

#if (CURRENT_LOOP_TEST == 2)
    /* ========== 纯ADC诊断模式 (不运行电流环) ========== */

    if (!ADC_Is_Started())
    {
        ADC_Current_Init();
    }
    HAL_Delay(200);

#elif (CURRENT_LOOP_TEST == 3)
    /* ========== 速度+电流双闭环模式 ========== */

    if (!ADC_Is_Started()) { ADC_Current_Init(); }
    HAL_Delay(100);

    /* 应用PID参数 */
    vel_Kp = M1_VEL_KP; vel_Ki = M1_VEL_KI; vel_Kd = M1_VEL_KD; vel_LPF_Tf = M1_VEL_LPF_TF;
    vel_m2_Kp = M2_VEL_KP; vel_m2_Ki = M2_VEL_KI; vel_m2_Kd = M2_VEL_KD; vel_m2_LPF_Tf = M2_VEL_LPF_TF;
    cur_m1_Kp = M1_CUR_KP; cur_m1_Ki = M1_CUR_KI; cur_m1_Kd = M1_CUR_KD; cur_m1_LPF_Tf = M1_CUR_LPF_TF;
    cur_m2_Kp = M2_CUR_KP; cur_m2_Ki = M2_CUR_KI; cur_m2_Kd = M2_CUR_KD; cur_m2_LPF_Tf = M2_CUR_LPF_TF;
    velocity_limit = VELOCITY_LIMIT;

    velocityCurrentClosedloop_M1_Init();
    velocityCurrentClosedloop_M2_Init();

    for (;;)
    {
        velocityCurrentClosedloop_M1(m1_target_velocity);
        velocityCurrentClosedloop_M2(m2_target_velocity);
        osDelay(1);
    }

#elif CURRENT_LOOP_TEST
    /* ========== 电流环测试模式 ========== */
    if (!ADC_Is_Started())
    {
        ADC_Current_Init();
    }
    HAL_Delay(100);

    cur_m1_Kp = M1_CUR_KP; cur_m1_Ki = M1_CUR_KI; cur_m1_Kd = M1_CUR_KD; cur_m1_LPF_Tf = M1_CUR_LPF_TF;
    cur_m2_Kp = M2_CUR_KP; cur_m2_Ki = M2_CUR_KI; cur_m2_Kd = M2_CUR_KD; cur_m2_LPF_Tf = M2_CUR_LPF_TF;

    currentClosedloop_M1_Init();
    currentClosedloop_M2_Init();

    for (;;)
    {
        currentClosedloop_M1(m1_target_current);
        currentClosedloop_M2(m2_target_current);
        osDelay(1);
    }

#else
    /* ========== 速度闭环模式 (原有功能) ========== */

    vel_Kp = M1_VEL_KP; vel_Ki = M1_VEL_KI; vel_Kd = M1_VEL_KD; vel_LPF_Tf = M1_VEL_LPF_TF;
    vel_m2_Kp = M2_VEL_KP; vel_m2_Ki = M2_VEL_KI; vel_m2_Kd = M2_VEL_KD; vel_m2_LPF_Tf = M2_VEL_LPF_TF;

    velocityClosedloop_Init();
    velocityClosedloop_M2_Init();

    for (;;)
    {
        velocityClosedloop(m1_target_velocity);
        velocityClosedloop_M2(m2_target_velocity);
        osDelay(1);
    }
#endif
}

/*============================================================================
 * UARTTask - 串口DMA接收双电机命令
 *   速度: A<rad/s>\n  B<rad/s>\n
 *   电流: C<安培>\n  D<安培>\n
 *   位置: E<rad>\n    F<rad>\n
 *   示例: A3.14\n  M1速度=3.14rad/s
 *         C0.1\n   M1电流=0.1A
 *         E6.28\n  M1角度=6.28rad (一圈)
 *============================================================================*/
void StartUARTTask(void *argument)
{
    uint8_t rx_cmd[UART_RX_BUF_SIZE];

    for (;;)
    {
        if (uart_rx_queue != NULL &&
            osMessageQueueGet(uart_rx_queue, rx_cmd, NULL, osWaitForever) == osOK)
        {
            rx_cmd[UART_RX_BUF_SIZE - 1] = '\0';
            char *cmd = (char *)rx_cmd;

            if (cmd[0] == 'A' || cmd[0] == 'a')
            {
                /* M1 速度命令 */
                float val = atof(cmd + 1);
                m1_target_velocity = val;
            }
            else if (cmd[0] == 'B' || cmd[0] == 'b')
            {
                /* M2 速度命令 */
                float val = atof(cmd + 1);
                m2_target_velocity = val;
            }
            else if (cmd[0] == 'C' || cmd[0] == 'c')
            {
                /* M1 电流命令 */
                float val = atof(cmd + 1);
                m1_target_current = val;
            }
            else if (cmd[0] == 'D' || cmd[0] == 'd')
            {
                /* M2 电流命令 */
                float val = atof(cmd + 1);
                m2_target_current = val;
            }
            else
            {
                UART_SendString("ERR: A/B(vel) C/D(cur)\r\n");
            }
        }
    }
}


/*============================================================================
 * OLEDTask - 双电机分屏显示 (50Hz)
 *   CURRENT_LOOP_TEST=0: 速度环显示 (T/N/E)
 *   CURRENT_LOOP_TEST=1: 电流环显示 (Tgt/Iq/Err)
 *============================================================================*/
void StartOLEDTask(void *argument)
{
    OLED_Init();

    char buf[16];

    for (;;)
    {
        OLED_NewFrame();

        if (oled_page == 0)
        {
            /* ========== 第1页: 电机状态 ========== */
            OLED_DrawLine(63, 0, 63, 63, OLED_COLOR_NORMAL);

            /* ---- 左半: M1 ---- */
            OLED_PrintASCIIString(0, 0, "M1", &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "S:%.2f", vel_actual_speed);
            OLED_PrintASCIIString(0, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "I:%.3f", cur_m1_actual_iq);
            OLED_PrintASCIIString(0, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

            /* 最后一行不显示任何内容 */

            /* ---- 右半: M2 ---- */
            OLED_PrintASCIIString(65, 0, "M2", &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "S:%.2f", vel_m2_actual_speed);
            OLED_PrintASCIIString(65, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "I:%.3f", cur_m2_actual_iq);
            OLED_PrintASCIIString(65, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

            /* 最后一行不显示任何内容 */
        }
        else
        {
            /* ========== 第2页: PID参数 ========== */
            OLED_DrawLine(63, 0, 63, 63, OLED_COLOR_NORMAL);

            /* ---- 左半: 速度环PID ---- */
            OLED_PrintASCIIString(0, 0, "VEL", &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "P:%.3f", vel_Kp);
            OLED_PrintASCIIString(0, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "I:%.3f", vel_Ki);
            OLED_PrintASCIIString(0, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "D:%.3f", vel_Kd);
            OLED_PrintASCIIString(0, 48, buf, &afont16x8, OLED_COLOR_NORMAL);

            /* ---- 右半: 电流环PID ---- */
            OLED_PrintASCIIString(65, 0, "CUR", &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "P:%.3f", cur_m1_Kp);
            OLED_PrintASCIIString(65, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "I:%.3f", cur_m1_Ki);
            OLED_PrintASCIIString(65, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

            sprintf(buf, "D:%.3f", cur_m1_Kd);
            OLED_PrintASCIIString(65, 48, buf, &afont16x8, OLED_COLOR_NORMAL);
        }

        OLED_ShowFrame();

        osDelay(20);
    }
}

/*============================================================================
 * ADCTestTask - ADC电流监控 (每秒打印)
 *   CURRENT_LOOP_TEST=0: 打印原始ADC值
 *   CURRENT_LOOP_TEST=1: 打印电流环实际电流值
 *============================================================================*/
void StartADCTestTask(void *argument)
{
    /* 等待系统稳定 */
    osDelay(2000);

    /* 初始化ADC */
    ADC_Current_Init();

    for (;;)
    {
#if (CURRENT_LOOP_TEST == 0)
        /* 速度环模式: 打印原始ADC + 电压 (监控电流传感器是否工作) */
        // 删除输出
#elif (CURRENT_LOOP_TEST == 4)
        /* 三环模式: 角度+速度+电流 */
        // 删除输出
#elif (CURRENT_LOOP_TEST == 3)
        /* 速度+电流双闭环: 目标速度+实际速度+电流 */
        // 删除输出
#elif (CURRENT_LOOP_TEST == 2)
        /* 诊断模式: 只打印原始ADC */
        // 删除输出
#elif CURRENT_LOOP_TEST
        /* 电流环模式: 显示实际电流值 + 原始ADC */
        // 删除输出
#else
        /* 速度环模式: 原始ADC */
        // 删除输出
#endif
        osDelay(1000);
    }
}

/*============================================================================
 * KeyTask - 按键检测任务 (10ms 周期)
 *   检测 PA0, PB0, PB1 三个按键
 *   功能待定, 当前仅打印按键事件
 *============================================================================*/
void StartKeyTask(void *argument)
{
    Key_Init();

    for (;;)
    {
        Key_Scan();

        for (int i = 0; i < KEY_NUM; i++)
        {
            KeyEvent_t event = Key_GetEvent((KeyId_t)i);

            if (event == KEY_EVENT_PRESS)
            {
                // 删除输出
            }
            else if (event == KEY_EVENT_RELEASE)
            {
                // 删除输出
            }
            else if (event == KEY_EVENT_LONG_PRESS)
            {
                // 删除输出

                /* KEY_1长按: 翻页 */
                if (i == KEY_1)
                {
                    oled_page ^= 1;
                }
            }
        }

        osDelay(10);
    }
}
