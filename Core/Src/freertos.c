/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  *                      - AS5600Task: 编码器读取 (1kHz)
  *                      - MotorTask: 速度/位置闭环控制 (1kHz)
  *                      - OLEDTask: 显示刷新 (50Hz)
  *                      - UARTTask: 串口命令解析
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
#include "encoder.h"
#include "PID.h"
#include "LPF.h"
#include "uart_comm.h"
#include "OLED.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* 电机运行模式 */
typedef enum {
    MODE_STOP = 0,      /* 停机 */
    MODE_VEL_OPENLOOP,  /* 速度开环 */
    MODE_VEL_CLOSEDLOOP,/* 速度闭环 */
    MODE_POS_CLOSEDLOOP /* 位置闭环 */
} MotorMode_t;

static volatile MotorMode_t motor_mode = MODE_STOP;
static volatile float target_velocity = 0.0f;   /* 目标速度 rad/s */
static volatile float target_position = 0.0f;   /* 目标角度 rad */

/* USER CODE END Variables */

/* Definitions for AS5600Task */
osThreadId_t AS5600TaskHandle;
const osThreadAttr_t AS5600Task_attributes = {
  .name = "AS5600Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,  /* 最高优先级, 保证I2C不被打断 */
};

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,  /* 低于AS5600Task */
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

  /* 创建任务 (按优先级从高到低) */
  AS5600TaskHandle = osThreadNew(StartAS5600Task, NULL, &AS5600Task_attributes);
  MotorTaskHandle  = osThreadNew(StartMotorTask,  NULL, &MotorTask_attributes);
  UARTTaskHandle   = osThreadNew(StartUARTTask,   NULL, &UARTTask_attributes);
  OLEDTaskHandle   = osThreadNew(StartOLEDTask,   NULL, &OLEDTask_attributes);

  /* 启动UART通信 */
  UART_Comm_Init();

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
}

/*============================================================================
 * AS5600Task - 编码器读取 (500Hz, 最高优先级)
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
                UART_Printf("AS5600 I2C fail\r\n");
                fail_cnt = 0;
            }
        }
        osDelay(2);  /* 2ms周期, 给AS5600足够转换时间 */
    }
}

/*============================================================================
 * MotorTask - 电机实时控制 (1kHz)
 * 支持: 停机 / 速度开环 / 速度闭环 / 位置闭环
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
    UART_Printf("AS5600 OK raw:%d\r\n", as5600_raw);

    /* 零电角度校准 (内部会重置AS5600和encoder) */
    UART_SendString("Aligning...\r\n");
    alignSensor();
    UART_Printf("Align done zero_el:%.4f\r\n", zero_electric_angle);

    /* 校准完成后, 等待一段时间让AS5600任务刷新几次数据 */
    osDelay(100);

    /* 初始化编码器模块 (用当前有效数据作为起点) */
    encoder_init();

    /* 初始化速度闭环 (PID + LPF) */
    velocityClosedloop_Init();

    /* 初始化位置闭环 (PID + LPF) */
    positionClosedloop_Init();

    /* 默认: 速度闭环模式, 目标速度 0 */
    motor_mode = MODE_VEL_CLOSEDLOOP;
    target_velocity = 0.0f;

    UART_Printf("Motor ready. mode:VEL\r\n");

    MotorMode_t mode_prev = MODE_STOP;

    for (;;)
    {
        /* ---- 0. 更新编码器 ---- */
        encoder_update();

        /* ---- 1. 根据模式执行控制 ---- */
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

        /* ---- 2. 模式切换时重新初始化 ---- */
        if (motor_mode != mode_prev)
        {
            if (motor_mode == MODE_VEL_CLOSEDLOOP)
            {
                velocityClosedloop_Init();
                UART_Printf("Switch to VEL_CLOSEDLOOP\r\n");
            }
            else if (motor_mode == MODE_POS_CLOSEDLOOP)
            {
                positionClosedloop_Init();
                target_position = encoder_get_angle();  /* 目标=当前位置 */
                UART_Printf("Switch to POS_CLOSEDLOOP\r\n");
            }
            else if (motor_mode == MODE_STOP)
            {
                setPhaseVoltage(0, 0, 0);
                UART_Printf("Motor stopped\r\n");
            }
            mode_prev = motor_mode;
        }

        osDelay(1);  /* 1ms控制周期 */
    }
}

/*============================================================================
 * UARTTask - 串口命令解析
 *
 * 命令列表:
 *   stop              - 停机
 *   vel               - 切换到速度闭环模式
 *   pos               - 切换到位置闭环模式
 *   open              - 切换到速度开环模式
 *   <数字>            - 设置目标速度 (rad/s) 或目标角度 (rad)
 *   vkp:<val>         - 速度环 Kp
 *   vki:<val>         - 速度环 Ki
 *   vlpf:<val>        - 速度低通滤波时间常数 (s)
 *   pkp:<val>         - 位置环 Kp
 *   pkd:<val>         - 位置环 Kd
 *   angle             - 查询当前角度
 *   vel               - 查询当前速度
 *   info              - 查询所有参数
 *============================================================================*/
void StartUARTTask(void *argument)
{
    uint8_t rx_cmd[UART_RX_BUF_SIZE];

    for (;;)
    {
        /* 阻塞等待UART命令 */
        if (uart_rx_queue != NULL &&
            osMessageQueueGet(uart_rx_queue, rx_cmd, NULL, osWaitForever) == osOK)
        {
            rx_cmd[63] = '\0';
            char *cmd = (char *)rx_cmd;

            /* ---- 模式切换命令 ---- */
            if (strcmp(cmd, "stop") == 0)
            {
                motor_mode = MODE_STOP;
                UART_Printf("OK stop\r\n");
            }
            else if (strcmp(cmd, "vel") == 0)
            {
                motor_mode = MODE_VEL_CLOSEDLOOP;
                UART_Printf("OK vel mode\r\n");
            }
            else if (strcmp(cmd, "pos") == 0)
            {
                motor_mode = MODE_POS_CLOSEDLOOP;
                target_position = encoder_get_angle();
                UART_Printf("OK pos mode\r\n");
            }
            else if (strcmp(cmd, "open") == 0)
            {
                motor_mode = MODE_VEL_OPENLOOP;
                UART_Printf("OK openloop mode\r\n");
            }
            /* ---- 速度PID参数 ---- */
            else if (strncmp(cmd, "vkp:", 4) == 0)
            {
                vel_Kp = atof(&cmd[4]);
                UART_Printf("OK vkp:%.4f\r\n", vel_Kp);
            }
            else if (strncmp(cmd, "vki:", 4) == 0)
            {
                vel_Ki = atof(&cmd[4]);
                UART_Printf("OK vki:%.4f\r\n", vel_Ki);
            }
            else if (strncmp(cmd, "vlpf:", 5) == 0)
            {
                vel_LPF_Tf = atof(&cmd[5]);
                UART_Printf("OK vlpf:%.4f\r\n", vel_LPF_Tf);
            }
            /* ---- 位置PID参数 ---- */
            else if (strncmp(cmd, "pkp:", 4) == 0)
            {
                pos_Kp = atof(&cmd[4]);
                UART_Printf("OK pkp:%.4f\r\n", pos_Kp);
            }
            else if (strncmp(cmd, "pkd:", 4) == 0)
            {
                pos_Kd = atof(&cmd[4]);
                UART_Printf("OK pkd:%.4f\r\n", pos_Kd);
            }
            /* ---- 查询命令 ---- */
            else if (strcmp(cmd, "angle") == 0)
            {
                float a = fmod(encoder_get_angle(), 2*PI);
                if (a < 0) a += 2*PI;
                UART_Printf("angle:%.4f raw:%d\r\n", a, as5600_raw);
            }
            else if (strcmp(cmd, "speed") == 0)
            {
                UART_Printf("speed:%.4f\r\n", vel_actual_speed);
            }
            else if (strcmp(cmd, "info") == 0)
            {
                const char *mode_str = "STOP";
                if (motor_mode == MODE_VEL_OPENLOOP)   mode_str = "VEL_OPEN";
                if (motor_mode == MODE_VEL_CLOSEDLOOP) mode_str = "VEL_PID";
                if (motor_mode == MODE_POS_CLOSEDLOOP) mode_str = "POS_PID";

                UART_Printf("mode:%s tgt_v:%.2f tgt_p:%.2f\r\n",
                            mode_str, target_velocity, target_position);
                UART_Printf("vel:%.2f vkp:%.3f vki:%.3f vlpf:%.3f\r\n",
                            vel_actual_speed, vel_Kp, vel_Ki, vel_LPF_Tf);
                UART_Printf("pos:%.4f pkp:%.2f pkd:%.2f\r\n",
                            encoder_get_angle(), pos_Kp, pos_Kd);
            }
            /* ---- 数字输入: 设置目标 ---- */
            else
            {
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
                        UART_Printf("OK vel:%.4f\r\n", target_velocity);
                    }
                }
            }
        }
    }
}

/*============================================================================
 * OLEDTask - 显示刷新 (50Hz)
 * 使用 OLED_NewFrame / OLED_PrintASCIIString / OLED_ShowFrame API
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
        float angle = fmod(encoder_get_angle(), 2*PI);
        if (angle < 0) angle += 2*PI;
        sprintf(buf, "Ang:%.2f", angle);
        OLED_PrintASCIIString(0, 48, buf, &afont16x8, OLED_COLOR_NORMAL);

        OLED_ShowFrame();

        osDelay(20);  /* 50Hz刷新 */
    }
}
