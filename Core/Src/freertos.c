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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* =========================================================================
 * 用户配置区 - 双电机PID参数
 * ========================================================================= */

/* M1 速度环 PID 参数 */
#define M1_VEL_KP        0.02f
#define M1_VEL_KI        0.05f
#define M1_VEL_KD        0.0f
#define M1_VEL_LPF_TF    0.4f

/* M2 速度环 PID 参数 */
#define M2_VEL_KP        0.02f
#define M2_VEL_KI        0.05f
#define M2_VEL_KD        0.0f
#define M2_VEL_LPF_TF    0.4f

/* ========================================================================= */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static volatile float m1_target_velocity = 0.0f;
static volatile float m2_target_velocity = 0.0f;

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

/* Function prototypes */
void StartAS5600Task(void *argument);
void StartAS5600M2Task(void *argument);
void StartMotorTask(void *argument);
void StartOLEDTask(void *argument);
void StartUARTTask(void *argument);

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

    if (as5600_ready)
        UART_SendString("M1 AS5600 ready\r\n");
    else
        UART_SendString("M1 AS5600 FAILED\r\n");

    for (;;)
    {
        if (!AS5600_Read())
        {
            static uint8_t fail_cnt = 0;
            if (++fail_cnt >= 20)
            {
                UART_SendString("M1 I2C fail\r\n");
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

    if (as5600_m2_ready)
        UART_SendString("M2 AS5600 ready\r\n");
    else
        UART_SendString("M2 AS5600 FAILED\r\n");

    for (;;)
    {
        if (!AS5600_M2_Read())
        {
            static uint8_t fail_cnt = 0;
            if (++fail_cnt >= 20)
            {
                UART_SendString("M2 I2C fail\r\n");
                fail_cnt = 0;
            }
        }
        osDelay(2);
    }
}

/*============================================================================
 * MotorTask - 双电机速度闭环控制 (1kHz)
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
    UART_SendString("Wait M1...\r\n");
    while (!as5600_ready) { osDelay(10); }
    UART_SendString("M1 OK\r\n");

    /* ---- 等待M2编码器就绪 ---- */
    UART_SendString("Wait M2...\r\n");
    while (!as5600_m2_ready) { osDelay(10); }
    UART_SendString("M2 OK\r\n");

    /* ---- M1 零电角度校准 ---- */
    UART_SendString("Align M1...\r\n");
    alignSensor();
    UART_SendString("M1 align done\r\n");

    /* ---- M2 零电角度校准 ---- */
    UART_SendString("Align M2...\r\n");
    alignSensor_M2();
    UART_SendString("M2 align done\r\n");

    osDelay(100);

    /* ---- 应用M1 PID参数 ---- */
    vel_Kp     = M1_VEL_KP;
    vel_Ki     = M1_VEL_KI;
    vel_Kd     = M1_VEL_KD;
    vel_LPF_Tf = M1_VEL_LPF_TF;

    /* ---- 应用M2 PID参数 ---- */
    vel_m2_Kp     = M2_VEL_KP;
    vel_m2_Ki     = M2_VEL_KI;
    vel_m2_Kd     = M2_VEL_KD;
    vel_m2_LPF_Tf = M2_VEL_LPF_TF;

    /* 初始化双电机闭环控制 */
    velocityClosedloop_Init();
    velocityClosedloop_M2_Init();

    UART_Printf("M1 kp:%.3f ki:%.3f\r\n", vel_Kp, vel_Ki);
    UART_Printf("M2 kp:%.3f ki:%.3f\r\n", vel_m2_Kp, vel_m2_Ki);

    for (;;)
    {
        /* M1 速度闭环 */
        velocityClosedloop(m1_target_velocity);

        /* M2 速度闭环 */
        velocityClosedloop_M2(m2_target_velocity);

        osDelay(1);
    }
}

/*============================================================================
 * UARTTask - 串口DMA接收双电机命令
 *   格式: A<速度>\n  设置M1目标速度
 *         B<速度>\n  设置M2目标速度
 *   示例: A10\n    M1目标=10 rad/s
 *         B-5.5\n  M2目标=-5.5 rad/s
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
                /* M1 命令 */
                float val = atof(cmd + 1);
                m1_target_velocity = val;
                UART_Printf("M1:%.2f\r\n", m1_target_velocity);
            }
            else if (cmd[0] == 'B' || cmd[0] == 'b')
            {
                /* M2 命令 */
                float val = atof(cmd + 1);
                m2_target_velocity = val;
                UART_Printf("M2:%.2f\r\n", m2_target_velocity);
            }
            else
            {
                UART_SendString("ERR: use A/B\r\n");
            }
        }
    }
}

/*============================================================================
 * OLEDTask - 双电机分屏显示 (50Hz)
 *   左半: M1参数  右半: M2参数
 *   每侧显示: T(目标), N(实际), E(误差)
 *============================================================================*/
void StartOLEDTask(void *argument)
{
    OLED_Init();

    char buf[12];

    for (;;)
    {
        OLED_NewFrame();

        /* ---- 中间分隔线 ---- */
        OLED_DrawLine(63, 0, 63, 63, OLED_COLOR_NORMAL);

        /* ---- 左半: M1 ---- */
        OLED_PrintASCIIString(0, 0, "M1", &afont16x8, OLED_COLOR_NORMAL);

        /* M1 T (目标) */
        sprintf(buf, "T:%.1f", m1_target_velocity);
        OLED_PrintASCIIString(0, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* M1 N (实际) */
        sprintf(buf, "N:%.1f", vel_actual_speed);
        OLED_PrintASCIIString(0, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* M1 E (误差) */
        sprintf(buf, "E:%.1f", m1_target_velocity - vel_actual_speed);
        OLED_PrintASCIIString(0, 48, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* ---- 右半: M2 ---- */
        OLED_PrintASCIIString(65, 0, "M2", &afont16x8, OLED_COLOR_NORMAL);

        /* M2 T (目标) */
        sprintf(buf, "T:%.1f", m2_target_velocity);
        OLED_PrintASCIIString(65, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* M2 N (实际) */
        sprintf(buf, "N:%.1f", vel_m2_actual_speed);
        OLED_PrintASCIIString(65, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* M2 E (误差) */
        sprintf(buf, "E:%.1f", m2_target_velocity - vel_m2_actual_speed);
        OLED_PrintASCIIString(65, 48, buf, &afont16x8, OLED_COLOR_NORMAL);

        OLED_ShowFrame();

        osDelay(20);
    }
}
