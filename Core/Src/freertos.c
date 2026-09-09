/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for AS5600Task */
osThreadId_t AS5600TaskHandle;
const osThreadAttr_t AS5600Task_attributes = {
  .name = "AS5600Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,  /* 高于MotorTask */
};

void StartMotorTask(void *argument);
void StartAS5600Task(void *argument);

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */
  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */
  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* 创建任务 */
  AS5600TaskHandle = osThreadNew(StartAS5600Task, NULL, &AS5600Task_attributes);
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* 启动UART通信 (创建消息队列 + 启动DMA接收) */
  UART_Comm_Init();

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

/**
  * @brief  AS5600Task - 专用于编码器读取
  *         以1kHz频率读取AS5600, 更新全局角度变量
  *         I2C失败时自动恢复总线
  */
void StartAS5600Task(void *argument)
{
  AS5600_Init();

  if (as5600_ready)
    UART_SendString("AS5600 ready\r\n");
  else
    UART_SendString("AS5600 init FAILED, retrying...\r\n");

  for(;;)
  {
    if (AS5600_Read())
    {
      /* 读取成功 */
    }
    else
    {
      /* 读取失败, I2C总线可能卡死, 已在AS5600_Read中处理 */
      static uint8_t fail_cnt = 0;
      if (++fail_cnt >= 50)  /* 每50次失败报告一次 */
      {
        UART_Printf("AS5600 I2C fail, raw:%d\r\n", as5600_raw);
        fail_cnt = 0;
      }
    }

    osDelay(1);  /* 1ms周期, 1kHz采样率 */
  }
}

/**
  * @brief  MotorTask - 位置闭环控制
  *
  *  串口发送 0~6.28 数字, 电机转到对应角度 (rad)
  *
  *  UART命令:
  *    <数字>     - 设置目标位置 (rad)
  *    pkp:<val>  - 调整位置环 Kp
  *    angle      - 查询当前角度
  *    info       - 查询当前参数
  */
void StartMotorTask(void *argument)
{
  /* ---- 硬件初始化 ---- */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  /* 初始化DWT周期计数器 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* 等待AS5600就绪 */
  UART_SendString("Waiting for AS5600...\r\n");
  while (!as5600_ready)
  {
    osDelay(10);
  }
  UART_Printf("AS5600 OK, raw:%d\r\n", as5600_raw);

  /* 零电角度校准 */
  UART_SendString("Aligning sensor...\r\n");
  alignSensor();
  UART_Printf("Align done, zero_el:%.4f\r\n", zero_electric_angle);

  /* 初始化位置闭环 */
  positionClosedloop_Init();

  /* ---- 任务变量 ---- */
  float    target_pos  = as5600_angle;  /* 初始目标 = 当前位置 */
  uint32_t report_tick = 0;
  uint8_t  rx_cmd[UART_RX_BUF_SIZE];

  UART_Printf("Motor ready. pos:%.4f pkp:%.2f\r\n", target_pos, pos_Kp);

  for(;;)
  {
    /* ---- 1. UART命令处理 ---- */
    if (uart_rx_queue != NULL &&
        osMessageQueueGet(uart_rx_queue, rx_cmd, NULL, 0) == osOK)
    {
      rx_cmd[63] = '\0';

      if (strncmp((char *)rx_cmd, "pkp:", 4) == 0)
      {
        pos_Kp = atof((char *)&rx_cmd[4]);
        UART_Printf("OK pkp:%.4f\r\n", pos_Kp);
      }
      else if (strncmp((char *)rx_cmd, "angle", 5) == 0)
      {
        float a = fmod(as5600_angle_single, 2*PI); if (a < 0) a += 2*PI;
        UART_Printf("angle:%.4f raw:%d\r\n", a, as5600_raw);
      }
      else if (strncmp((char *)rx_cmd, "info", 4) == 0)
      {
        float t = fmod(target_pos, 2*PI);       if (t < 0) t += 2*PI;
        float a = fmod(as5600_angle_single, 2*PI); if (a < 0) a += 2*PI;
        UART_Printf("target:%.2f actual:%.2f pkp:%.2f\r\n", t, a, pos_Kp);
      }
      else
      {
        /* 尝试解析为数字 */
        float val = atof((char *)rx_cmd);
        if (val != 0.0f || rx_cmd[0] == '0')
        {
          target_pos = val;
          UART_Printf("OK pos:%.4f\r\n", target_pos);
        }
      }
    }

    /* ---- 2. 位置闭环控制 ---- */
    pos_actual_angle = as5600_angle;  /* 从全局变量读取 */

    /* 我们的as5600_angle始终是正值, 不需要乘DIR
     * DIR只在getElectricalAngle里用于电角度计算 */
    float error_deg = (target_pos - pos_actual_angle) * 180.0f / PI;
    float Uq = pos_Kp * error_deg;
    if (Uq > pos_Uq_max) Uq = pos_Uq_max;
    if (Uq < -pos_Uq_max) Uq = -pos_Uq_max;

    float angle_el = getElectricalAngle();
    setPhaseVoltage(Uq, 0, angle_el);

    /* ---- 3. 每200ms上报详细调试信息 ---- */
    if (osKernelGetTickCount() - report_tick >= 200)
    {
      report_tick = osKernelGetTickCount();

      float t = fmod(target_pos, 2*PI);       if (t < 0) t += 2*PI;
      float a = fmod(pos_actual_angle, 2*PI); if (a < 0) a += 2*PI;

      UART_Printf("T:%.2f A:%.2f Err:%.1f Uq:%.2f el:%.2f raw:%d\r\n",
                  t, a, error_deg, Uq, angle_el, as5600_raw);
    }

    osDelay(1);
  }
}
