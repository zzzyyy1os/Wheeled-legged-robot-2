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
  *                      - SPITask:       SPI2从机接收F103-N命令
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
#include "spi_slave.h"
#include "OLED.h"
#include "adc_current.h"
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

/* M1 电流环 PID 参数 (降低增益减少振荡, 增大LPF减少噪声) */
#define M1_CUR_KP        1.0f
#define M1_CUR_KI        50.0f
#define M1_CUR_KD        0.0f
#define M1_CUR_LPF_TF    0.05f

/* M2 电流环 PID 参数 */
#define M2_CUR_KP        1.0f
#define M2_CUR_KI        50.0f
#define M2_CUR_KD        0.0f
#define M2_CUR_LPF_TF    0.05f

/* 控制模式选择: 0=速度环, 1=电流环测试, 2=纯ADC诊断 */
#define CURRENT_LOOP_TEST  1

/* ========================================================================= */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static volatile float m1_target_velocity = 0.0f;
static volatile float m2_target_velocity = 0.0f;

/* 电流环目标 (安培) */
static volatile float m1_target_current = 0.1f;   /* 默认0.1A */
static volatile float m2_target_current = 0.1f;

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

/* Definitions for SPITask */
osThreadId_t SPITaskHandle;
const osThreadAttr_t SPITask_attributes = {
  .name = "SPITask",
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

/* Function prototypes */
void StartAS5600Task(void *argument);
void StartAS5600M2Task(void *argument);
void StartMotorTask(void *argument);
void StartOLEDTask(void *argument);
void StartUARTTask(void *argument);
void StartSPITask(void *argument);
void StartADCTestTask(void *argument);

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
  SPITaskHandle      = osThreadNew(StartSPITask,      NULL, &SPITask_attributes);
  ADCTestTaskHandle  = osThreadNew(StartADCTestTask,  NULL, &ADCTestTask_attributes);

  UART_Comm_Init();
  SPI_Slave_Init();

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

    /* ---- 关键: 对齐后停止所有PWM输出, 等待电流降为0 ---- */
    setPhaseVoltage(0, 0, 0);
    setPhaseVoltage_M2(0, 0, 0);
    osDelay(1000);  /* 等待1秒让电机完全静止 */

#if (CURRENT_LOOP_TEST == 2)
    /* ========== 纯ADC诊断模式 (不运行电流环) ========== */
    UART_SendString("=== ADC DIAGNOSTIC MODE ===\r\n");

    if (!ADC_Is_Started())
    {
        UART_SendString("ADC not started, init now...\r\n");
        ADC_Current_Init();
    }
    HAL_Delay(200);

    /* 打印原始ADC值 */
    UART_Printf("Raw ADC: %u %u %u %u\r\n",
        adc_dma_buf[0], adc_dma_buf[1],
        adc_dma_buf[2], adc_dma_buf[3]);

    /* 打印电压值 */
    UART_Printf("Voltage: %.3fV %.3fV %.3fV %.3fV\r\n",
        adc_dma_buf[0]*ADC_CONV, adc_dma_buf[1]*ADC_CONV,
        adc_dma_buf[2]*ADC_CONV, adc_dma_buf[3]*ADC_CONV);

    /* 校准并打印偏移 */
    Current_Sensor_t test_m1 = { .Sen_Num = 0 };
    Current_Sensor_t test_m2 = { .Sen_Num = 1 };
    CurrSense_Init(&test_m1);
    CurrSense_Init(&test_m2);

    UART_Printf("M1 offset: %.4fV(ADC %d) %.4fV(ADC %d)\r\n",
        test_m1.offset_ia, (int)(test_m1.offset_ia/ADC_CONV),
        test_m1.offset_ib, (int)(test_m1.offset_ib/ADC_CONV));
    UART_Printf("M2 offset: %.4fV(ADC %d) %.4fV(ADC %d)\r\n",
        test_m2.offset_ia, (int)(test_m2.offset_ia/ADC_CONV),
        test_m2.offset_ib, (int)(test_m2.offset_ib/ADC_CONV));

    /* 持续打印ADC值 */
    for (;;)
    {
        UART_Printf("ADC: %u %u %u %u\r\n",
            adc_dma_buf[0], adc_dma_buf[1],
            adc_dma_buf[2], adc_dma_buf[3]);
        osDelay(500);
    }

#elif CURRENT_LOOP_TEST
    /* ========== 电流环测试模式 ========== */
    UART_SendString("=== CURRENT LOOP MODE ===\r\n");

    /* 关键: 确保ADC已启动 (ADCTestTask可能还没初始化) */
    if (!ADC_Is_Started())
    {
        UART_SendString("ADC not started, init now...\r\n");
        ADC_Current_Init();
    }
    HAL_Delay(100);  /* 等待DMA稳定 */
    UART_Printf("ADC buf: %u %u %u %u\r\n",
        adc_dma_buf[0], adc_dma_buf[1], adc_dma_buf[2], adc_dma_buf[3]);

    /* 应用电流环PID参数 */
    cur_m1_Kp     = M1_CUR_KP;
    cur_m1_Ki     = M1_CUR_KI;
    cur_m1_Kd     = M1_CUR_KD;
    cur_m1_LPF_Tf = M1_CUR_LPF_TF;

    cur_m2_Kp     = M2_CUR_KP;
    cur_m2_Ki     = M2_CUR_KI;
    cur_m2_Kd     = M2_CUR_KD;
    cur_m2_LPF_Tf = M2_CUR_LPF_TF;

    /* 初始化电流传感器 (偏移校准, 电机必须静止!) */
    UART_SendString("Calibrate M1 current (gain=-1)...\r\n");
    currentClosedloop_M1_Init();
    UART_Printf("M1 offset: %.4fV %.4fV\r\n",
        cur_m1_offset_ia, cur_m1_offset_ib);

    UART_SendString("Calibrate M2 current (gain=+1)...\r\n");
    currentClosedloop_M2_Init();
    UART_Printf("M2 offset: %.4fV %.4fV\r\n",
        cur_m2_offset_ia, cur_m2_offset_ib);

    UART_Printf("M1 cur kp:%.1f ki:%.1f\r\n", cur_m1_Kp, cur_m1_Ki);
    UART_Printf("M2 cur kp:%.1f ki:%.1f\r\n", cur_m2_Kp, cur_m2_Ki);

    for (;;)
    {
        /* M1 电流闭环 */
        currentClosedloop_M1(m1_target_current);

        /* M2 电流闭环 */
        currentClosedloop_M2(m2_target_current);

        osDelay(1);
    }

#else
    /* ========== 速度闭环模式 (原有功能) ========== */
    UART_SendString("=== VELOCITY LOOP MODE ===\r\n");

    /* 应用M1 PID参数 */
    vel_Kp     = M1_VEL_KP;
    vel_Ki     = M1_VEL_KI;
    vel_Kd     = M1_VEL_KD;
    vel_LPF_Tf = M1_VEL_LPF_TF;

    /* 应用M2 PID参数 */
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
#endif
}

/*============================================================================
 * UARTTask - 串口DMA接收双电机命令
 *   速度环模式: A<速度>\n  B<速度>\n
 *   电流环模式: C<电流>\n  D<电流>\n  (单位: 安培)
 *   示例: C0.1\n  M1目标=0.1A
 *         D-0.05\n M2目标=-0.05A
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
                UART_Printf("M1 vel:%.2f\r\n", m1_target_velocity);
            }
            else if (cmd[0] == 'B' || cmd[0] == 'b')
            {
                /* M2 速度命令 */
                float val = atof(cmd + 1);
                m2_target_velocity = val;
                UART_Printf("M2 vel:%.2f\r\n", m2_target_velocity);
            }
            else if (cmd[0] == 'C' || cmd[0] == 'c')
            {
                /* M1 电流命令 */
                float val = atof(cmd + 1);
                m1_target_current = val;
                UART_Printf("M1 cur:%.3fA\r\n", m1_target_current);
            }
            else if (cmd[0] == 'D' || cmd[0] == 'd')
            {
                /* M2 电流命令 */
                float val = atof(cmd + 1);
                m2_target_current = val;
                UART_Printf("M2 cur:%.3fA\r\n", m2_target_current);
            }
            else
            {
                UART_SendString("ERR: use A/B(vel) C/D(cur)\r\n");
            }
        }
    }
}

/*============================================================================
 * SPITask - SPI2从机接收F103-N命令
 *   帧格式: [0]=0xAA [1]=CMD [2..5]=Data(BE) [6]=XOR [7]=0x55
 *   命令: 0x01=心跳, 0x02=MPU数据, 0x03=M1速度, 0x04=M2速度
 *   速度编码: float×10 → int32 (大端序)
 *
 *   流程: SPI_Slave_WaitAndProcess() 阻塞等待帧→校验→入队
 *         然后从队列取出命令→更新电机目标速度
 *============================================================================*/
void StartSPITask(void *argument)
{
    SPI_RxItem_t rx_item;

    for (;;)
    {
        /* 阻塞等待SPI帧接收完成 (中断回调释放信号量) */
        SPI_Slave_WaitAndProcess();

        /* 从队列取出刚收到的命令并处理 */
        while (spiRxQueueHandle != NULL &&
               osMessageQueueGet(spiRxQueueHandle, &rx_item, NULL, 0) == osOK)
        {
            switch (rx_item.cmd)
            {
                case SPI_CMD_M1_VEL:
                {
                    m1_target_velocity = (float)rx_item.data / 10.0f;
                    break;
                }
                case SPI_CMD_M2_VEL:
                {
                    m2_target_velocity = (float)rx_item.data / 10.0f;
                    break;
                }
                case SPI_CMD_HEARTBEAT:
                case SPI_CMD_MPU_DATA:
                default:
                    break;
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

        /* ---- 中间分隔线 ---- */
        OLED_DrawLine(63, 0, 63, 63, OLED_COLOR_NORMAL);

#if CURRENT_LOOP_TEST
        /* ========== 电流环模式显示 ========== */

        /* ---- 左半: M1 ---- */
        OLED_PrintASCIIString(0, 0, "M1", &afont16x8, OLED_COLOR_NORMAL);

        /* M1 Tgt (目标电流) */
        sprintf(buf, "T:%.3f", m1_target_current);
        OLED_PrintASCIIString(0, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* M1 Iq (实际电流) */
        sprintf(buf, "I:%.3f", cur_m1_actual_iq);
        OLED_PrintASCIIString(0, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* M1 Err (误差) */
        sprintf(buf, "E:%.3f", m1_target_current - cur_m1_actual_iq);
        OLED_PrintASCIIString(0, 48, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* ---- 右半: M2 ---- */
        OLED_PrintASCIIString(65, 0, "M2", &afont16x8, OLED_COLOR_NORMAL);

        /* M2 Tgt (目标电流) */
        sprintf(buf, "T:%.3f", m2_target_current);
        OLED_PrintASCIIString(65, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* M2 Iq (实际电流) */
        sprintf(buf, "I:%.3f", cur_m2_actual_iq);
        OLED_PrintASCIIString(65, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* M2 Err (误差) */
        sprintf(buf, "E:%.3f", m2_target_current - cur_m2_actual_iq);
        OLED_PrintASCIIString(65, 48, buf, &afont16x8, OLED_COLOR_NORMAL);

#else
        /* ========== 速度环模式显示 ========== */

        /* ---- 左半: M1 ---- */
        OLED_PrintASCIIString(0, 0, "M1", &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "T:%.1f", m1_target_velocity);
        OLED_PrintASCIIString(0, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "N:%.1f", vel_actual_speed);
        OLED_PrintASCIIString(0, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "E:%.1f", m1_target_velocity - vel_actual_speed);
        OLED_PrintASCIIString(0, 48, buf, &afont16x8, OLED_COLOR_NORMAL);

        /* ---- 右半: M2 ---- */
        OLED_PrintASCIIString(65, 0, "M2", &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "T:%.1f", m2_target_velocity);
        OLED_PrintASCIIString(65, 16, buf, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "N:%.1f", vel_m2_actual_speed);
        OLED_PrintASCIIString(65, 32, buf, &afont16x8, OLED_COLOR_NORMAL);

        sprintf(buf, "E:%.1f", m2_target_velocity - vel_m2_actual_speed);
        OLED_PrintASCIIString(65, 48, buf, &afont16x8, OLED_COLOR_NORMAL);
#endif

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

    UART_SendString("ADC init done\r\n");

    for (;;)
    {
#if (CURRENT_LOOP_TEST == 2)
        /* 诊断模式: 只打印原始ADC */
        UART_Printf("ADC: %u %u %u %u\r\n",
            adc_dma_buf[0], adc_dma_buf[1],
            adc_dma_buf[2], adc_dma_buf[3]);
#elif CURRENT_LOOP_TEST
        /* 电流环模式: 显示实际电流值 + 原始ADC */
        UART_Printf("Iq M1:%.3fA M2:%.3fA | Raw:%u %u %u %u\r\n",
            cur_m1_actual_iq, cur_m2_actual_iq,
            adc_dma_buf[0], adc_dma_buf[1],
            adc_dma_buf[2], adc_dma_buf[3]);
#else
        /* 速度环模式: 原始ADC */
        UART_Printf("ADC: %u %u %u %u\r\n",
            adc_dma_buf[0], adc_dma_buf[1],
            adc_dma_buf[2], adc_dma_buf[3]);
#endif
        osDelay(1000);
    }
}
