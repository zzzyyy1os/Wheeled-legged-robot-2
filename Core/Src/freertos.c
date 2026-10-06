/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : 双电机FOC速度闭环控制
  *                      - AS5600Task:    M1编码器读取 (I2C3)
  *                      - AS5600M2Task:  M2编码器读取 (I2C2)
  *                      - MotorTask:     M1+M2速度闭环控制
  *                      - OLEDTask:      双电机状态显示 (50Hz, 软件I2C: PD2=SCL, PD3=SDA)
  *                      - MPU6050Task:   六轴传感器读取, 串口1打印 (500ms间隔)
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
#include "usart6.h"
#include "OLED.h"  /* 软件I2C: PD2=SCL, PD3=SDA */
#include "mpu6050.h"
#include "adc_current.h"
#include "key.h"
#include "servo.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* =========================================================================
 * 用户配置区 - 双电机PID参数
 * ========================================================================= */

/* M1 速度环 PID 参数 */
#define M1_VEL_KP        0.1f
#define M1_VEL_KI        0.1f
#define M1_VEL_KD        0.0f
#define M1_VEL_LPF_TF    0.2f

/* M2 速度环 PID 参数 */
#define M2_VEL_KP        0.1f
#define M2_VEL_KI        0.1f
#define M2_VEL_KD        0.0f
#define M2_VEL_LPF_TF    0.2f

/* M1 电流环 PID 参数 */
#define M1_CUR_KP        1.3f
#define M1_CUR_KI        0.4f
#define M1_CUR_KD        0.0f
#define M1_CUR_LPF_TF    0.2f

/* M2 电流环 PID 参数 */
#define M2_CUR_KP        1.3f
#define M2_CUR_KI        0.4f
#define M2_CUR_KD        0.0f
#define M2_CUR_LPF_TF    0.2f

/* 目标速度上限 (rad/s), 速度环输出(电流)由PID内部LIMIT=6.3限幅 */
#define VELOCITY_LIMIT   30.0f

/* 控制模式选择: 0=速度环, 1=电流环, 2=ADC诊断, 3=速度+电流双闭环, 4=三环嵌套 */
#define CURRENT_LOOP_TEST  3

/* ========================================================================= */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static volatile float m1_target_velocity = 0.0f;
static volatile float m2_target_velocity = 0.0f;

/* 电流环目标 (安培) */
static volatile float m1_target_current = 0.0f;   /* 默认0A, 通过串口C/D命令设置 */
static volatile float m2_target_current = 0.0f;

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

/* Definitions for OLEDTask (软件I2C: PD2=SCL, PD3=SDA) */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for MPU6050Task */
osThreadId_t MPU6050TaskHandle;
const osThreadAttr_t MPU6050Task_attributes = {
  .name = "MPU6050Task",
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

/* 舵机控制任务 */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Function prototypes */
void StartAS5600Task(void *argument);
void StartAS5600M2Task(void *argument);
void StartMotorTask(void *argument);
void StartOLEDTask(void *argument);
void StartMPU6050Task(void *argument);
void StartUARTTask(void *argument);
void StartADCTestTask(void *argument);
void StartKeyTask(void *argument);
void StartServoTask(void *argument);

/* USER CODE BEGIN Init */
/* USER CODE END Init */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* DWT微秒定时器 (必须在编码器任务之前初始化, 因为速度计算要用) */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  AS5600TaskHandle   = osThreadNew(StartAS5600Task,   NULL, &AS5600Task_attributes);
  AS5600M2TaskHandle = osThreadNew(StartAS5600M2Task, NULL, &AS5600M2Task_attributes);
  MotorTaskHandle    = osThreadNew(StartMotorTask,    NULL, &MotorTask_attributes);
  UARTTaskHandle     = osThreadNew(StartUARTTask,     NULL, &UARTTask_attributes);
  OLEDTaskHandle     = osThreadNew(StartOLEDTask,     NULL, &OLEDTask_attributes);
  MPU6050TaskHandle  = osThreadNew(StartMPU6050Task,   NULL, &MPU6050Task_attributes);
  ADCTestTaskHandle  = osThreadNew(StartADCTestTask,  NULL, &ADCTestTask_attributes);
  KeyTaskHandle      = osThreadNew(StartKeyTask,       NULL, &KeyTask_attributes);
  ServoTaskHandle    = osThreadNew(StartServoTask,     NULL, &ServoTask_attributes);

  UART_Comm_Init();
  MX_USART6_UART_Init();

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

    /* DWT已在MX_FREERTOS_Init中初始化 */

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
    osDelay(100);  /* 等待电机完全静止 */

    /* 设置系统就绪标志并发送就绪通知 */
    system_ready = 1;
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
    HAL_Delay(20);

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
 * UARTTask - 串口DMA接收命令
 *   电机: A<rad/s>  B<rad/s>  C<安培>  D<安培>
 *   舵机: U<0~100>  I<0~100>  O<0~100>  P<0~100>
 *         U=CH1, I=CH2, O=CH3, P=CH4
 *   示例: U50    舵机CH1设为50 (7.5%占空比)
 *         I100   舵机CH2设为100 (12.5%占空比)
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
            /* ---- 舵机命令: U/I/O/P → CH1~CH4 ---- */
            else if (cmd[0] == 'U' || cmd[0] == 'u')
            {
                float val = atof(cmd + 1);
                Servo_SetDuty(SERVO_CH1, val);
            }
            else if (cmd[0] == 'I' || cmd[0] == 'i')
            {
                float val = atof(cmd + 1);
                Servo_SetDuty(SERVO_CH2, val);
            }
            else if (cmd[0] == 'O' || cmd[0] == 'o')
            {
                float val = atof(cmd + 1);
                Servo_SetDuty(SERVO_CH3, val);
            }
            else if (cmd[0] == 'P' || cmd[0] == 'p')
            {
                float val = atof(cmd + 1);
                Servo_SetDuty(SERVO_CH4, val);
            }
            else
            {
                // 不发送错误信息
            }
        }
    }
}


/*============================================================================
 * OLEDTask - 双电机状态显示 (50Hz, 软件I2C: PD2=SCL, PD3=SDA)
 *============================================================================*/
void StartOLEDTask(void *argument)
{
    OLED_GPIO_Init();  /* 软件I2C引脚初始化 */
    OLED_Init();

    char buf[16];

    for (;;)
    {
        OLED_NewFrame();

        /* ========== 电机状态显示 ========== */
        OLED_DrawLine(63, 0, 63, 63, OLED_COLOR_NORMAL);

        /* ---- 左半: M1 ---- */
        OLED_PrintASCIIString(0, 0, "M1", &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "S:%.2f", vel_actual_speed);
        OLED_PrintASCIIString(0, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "I:%.3f", cur_m1_actual_iq);
        OLED_PrintASCIIString(0, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        OLED_PrintASCIIString(0, 48, "  ", &afont16x8, OLED_COLOR_NORMAL);

        /* ---- 右半: M2 ---- */
        OLED_PrintASCIIString(65, 0, "M2", &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "S:%.2f", vel_m2_actual_speed);
        OLED_PrintASCIIString(65, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "I:%.3f", cur_m2_actual_iq);
        OLED_PrintASCIIString(65, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        OLED_PrintASCIIString(65, 48, "  ", &afont16x8, OLED_COLOR_NORMAL);

        OLED_ShowFrame();

        osDelay(20);  /* 50Hz */
    }
}

/*============================================================================
 * MPU6050Task - 六轴传感器读取 (200Hz, 5ms周期)
 *   通过I2C1(PB6/PB7)读取MPU6050数据
 *   互补滤波融合 roll/pitch/yaw
 *   通过USART1打印到电脑 (500ms间隔)
 *============================================================================*/
void StartMPU6050Task(void *argument)
{
    /* 等待系统稳定 */
    osDelay(500);

    /* 初始化MPU6050 */
    MPU6050_Init();

    uint32_t last_print_time = 0;

    for (;;)
    {
        /* 读取并融合数据 (200Hz) */
        MPU6050_Read_Result();

        /* 每500ms打印一次MPU6050数据 */
        uint32_t current_time = osKernelGetTickCount();
        if (current_time - last_print_time >= 500)
        {
            last_print_time = current_time;

            /* 打印欧拉角和加速度 */
            UART_Printf("R:%.1f P:%.1f Y:%.1f Ax:%.2f Ay:%.2f Az:%.2f\r\n",
                        mpu6050_data.roll, mpu6050_data.pitch, mpu6050_data.yaw,
                        mpu6050_data.Ax, mpu6050_data.Ay, mpu6050_data.Az);
        }

        osDelay(5);  /* 200Hz */
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
            }
        }

        osDelay(10);
    }
}

/*============================================================================
 * ServoTask - 舵机PWM初始化 (TIM4 300Hz, PD12~PD15)
 *   串口命令 U/I/O/P 在 UARTTask 中处理, 调用 Servo_SetDuty()
 *============================================================================*/
void StartServoTask(void *argument)
{
    /* 等待系统稳定 */
    osDelay(500);

    /* 初始化舵机PWM (TIM4 300Hz) */
    Servo_Init();

    /* 舵机由 UARTTask 通过 Servo_SetDuty() 控制, 此任务空闲 */
    for (;;)
    {
        osDelay(1000);
    }
}
