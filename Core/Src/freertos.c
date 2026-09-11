/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FOC电机控制
  *                      - AS5600Task: 编码器读取
  *                      - MotorTask: 速度/位置闭环控制
  *                      - OLEDTask: 显示刷新
  *                      - UARTTask: 接收目标值
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
#include "uart_comm.h"
#include "OLED.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* =========================================================================
 * 用户配置区 - 在这里修改电机模式和PID参数
 * ========================================================================= */

/* 电机运行模式 (取消注释你想要的模式, 只能选一个) */
// #define MOTOR_MODE_POS_CLOSEDLOOP   /* 位置闭环 */
#define MOTOR_MODE_VEL_CLOSEDLOOP   /* 速度闭环 */

/* 速度环 PID 参数 */
#define VEL_KP        0.02f
#define VEL_KI        0.05f
#define VEL_KD        0.0f
#define VEL_LPF_TF    0.4f

/* 位置环 PID 参数 */
#define POS_KP        0.133f
#define POS_KI        0.01f
#define POS_KD        0.0f

/* ========================================================================= */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

typedef enum {
    MODE_STOP = 0,
    MODE_VEL_OPENLOOP,
    MODE_VEL_CLOSEDLOOP,
    MODE_POS_CLOSEDLOOP
} MotorMode_t;

/* 根据宏定义确定运行模式 */
#ifdef MOTOR_MODE_POS_CLOSEDLOOP
static volatile MotorMode_t motor_mode = MODE_POS_CLOSEDLOOP;
#else
static volatile MotorMode_t motor_mode = MODE_VEL_CLOSEDLOOP;
#endif

static volatile float target_velocity = 0.0f;
static volatile float target_position = 0.0f;

/* USER CODE END Variables */

/* Definitions for AS5600Task */
osThreadId_t AS5600TaskHandle;
const osThreadAttr_t AS5600Task_attributes = {
  .name = "AS5600Task",
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
void StartMotorTask(void *argument);
void StartOLEDTask(void *argument);
void StartUARTTask(void *argument);

/* USER CODE BEGIN Init */
/* USER CODE END Init */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  AS5600TaskHandle = osThreadNew(StartAS5600Task, NULL, &AS5600Task_attributes);
  MotorTaskHandle  = osThreadNew(StartMotorTask,  NULL, &MotorTask_attributes);
  UARTTaskHandle   = osThreadNew(StartUARTTask,   NULL, &UARTTask_attributes);
  OLEDTaskHandle   = osThreadNew(StartOLEDTask,   NULL, &OLEDTask_attributes);

  UART_Comm_Init();

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
}

/*============================================================================
 * AS5600Task - 编码器读取 (500Hz)
 *============================================================================*/
void StartAS5600Task(void *argument)
{
    AS5600_Init();

    if (as5600_ready)
        UART_SendString("AS5600 ready\r\n");
    else
        UART_SendString("AS5600 init FAILED\r\n");

    for (;;)
    {
        if (!AS5600_Read())
        {
            static uint8_t fail_cnt = 0;
            if (++fail_cnt >= 20)
            {
                UART_SendString("AS5600 I2C fail\r\n");
                fail_cnt = 0;
            }
        }
        osDelay(2);
    }
}

/*============================================================================
 * MotorTask - 电机实时控制 (1kHz)
 *============================================================================*/
void StartMotorTask(void *argument)
{
    /* ---- 硬件初始化 ---- */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* DWT微秒定时器 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 等待AS5600就绪 */
    UART_SendString("Waiting AS5600...\r\n");
    while (!as5600_ready) { osDelay(10); }
    UART_Printf("AS5600 OK\r\n");

    /* 零电角度校准 */
    UART_SendString("Aligning...\r\n");
    alignSensor();
    UART_Printf("Align done\r\n");

    osDelay(100);

    /* ---- 应用宏定义的PID参数 ---- */
    vel_Kp     = VEL_KP;
    vel_Ki     = VEL_KI;
    vel_Kd     = VEL_KD;
    vel_LPF_Tf = VEL_LPF_TF;
    pos_Kp     = POS_KP;
    pos_Ki     = POS_KI;
    pos_Kd     = POS_KD;

    /* 初始化闭环控制 */
    velocityClosedloop_Init();
    positionClosedloop_Init();

    /* 打印当前配置 */
#ifdef MOTOR_MODE_POS_CLOSEDLOOP
    motor_mode = MODE_POS_CLOSEDLOOP;
    UART_Printf("Motor ready. POS mode pkp:%.3f\r\n", pos_Kp);
#else
    motor_mode = MODE_VEL_CLOSEDLOOP;
    UART_Printf("Motor ready. VEL mode kp:%.3f ki:%.3f\r\n", vel_Kp, vel_Ki);
#endif

    MotorMode_t mode_prev = MODE_STOP;

    for (;;)
    {
        /* ---- 执行控制 ---- */
        switch (motor_mode)
        {
            case MODE_STOP:
                setPhaseVoltage(0, 0, 0);
                break;
            case MODE_VEL_OPENLOOP:
                velocityOpenloop(target_velocity);
                break;
            case MODE_VEL_CLOSEDLOOP:
                velocityClosedloop(target_velocity);
                break;
            case MODE_POS_CLOSEDLOOP:
                positionClosedloop(target_position);
                break;
            default:
                setPhaseVoltage(0, 0, 0);
                break;
        }

        /* ---- 模式切换时重新初始化 ---- */
        if (motor_mode != mode_prev)
        {
            if (motor_mode == MODE_VEL_CLOSEDLOOP)
                velocityClosedloop_Init();
            else if (motor_mode == MODE_POS_CLOSEDLOOP)
            {
                positionClosedloop_Init();
                target_position = GetAngle();
            }
            else if (motor_mode == MODE_STOP)
                setPhaseVoltage(0, 0, 0);
            mode_prev = motor_mode;
        }

        osDelay(1);
    }
}

/*============================================================================
 * UARTTask - 串口接收目标值
 *   速度模式: 发送数字设置目标速度 (rad/s)
 *   位置模式: 发送数字设置目标角度 (rad)
 *============================================================================*/
void StartUARTTask(void *argument)
{
    uint8_t rx_cmd[UART_RX_BUF_SIZE];

    for (;;)
    {
        if (uart_rx_queue != NULL &&
            osMessageQueueGet(uart_rx_queue, rx_cmd, NULL, osWaitForever) == osOK)
        {
            rx_cmd[63] = '\0';
            char *cmd = (char *)rx_cmd;

            /* 解析数字 */
            float val = atof(cmd);
            if (val != 0.0f || cmd[0] == '0')
            {
                if (motor_mode == MODE_POS_CLOSEDLOOP)
                {
                    target_position = val;
                    UART_Printf("OK pos:%.4f\r\n", target_position);
                }
                else
                {
                    target_velocity = val;
                    UART_Printf("OK vel:%.2f\r\n", target_velocity);
                }
            }
        }
    }
}

/*============================================================================
 * OLEDTask - 显示刷新 (50Hz)
 *============================================================================*/
void StartOLEDTask(void *argument)
{
    OLED_Init();

    char buf[24];

    for (;;)
    {
        OLED_NewFrame();

        /* 第一行: 模式 */
        const char *mode_str = "STOP";
        if (motor_mode == MODE_VEL_OPENLOOP)   mode_str = "V_OPEN";
        if (motor_mode == MODE_VEL_CLOSEDLOOP) mode_str = "V_PID";
        if (motor_mode == MODE_POS_CLOSEDLOOP) mode_str = "P_PID";
        sprintf(buf, "FOC %s", mode_str);
        OLED_PrintASCIIString(0, 0, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* 第二行: 目标 */
        if (motor_mode == MODE_POS_CLOSEDLOOP)
        {
            float t = fmod(target_position, 2*PI);
            if (t < 0) t += 2*PI;
            sprintf(buf, "Tgt:%.2f", t);
        }
        else
        {
            sprintf(buf, "Tgt:%.2f", target_velocity);
        }
        OLED_PrintASCIIString(0, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* 第三行: 实际速度 */
        sprintf(buf, "Vel:%.2f", vel_actual_speed);
        OLED_PrintASCIIString(0, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* 第四行: 角度 */
        float angle = fmod(GetAngle(), 2*PI);
        if (angle < 0) angle += 2*PI;
        sprintf(buf, "Ang:%.2f", angle);
        OLED_PrintASCIIString(0, 48, buf, &afont16x8, OLED_COLOR_NORMAL);

        OLED_ShowFrame();

        osDelay(20);
    }
}
