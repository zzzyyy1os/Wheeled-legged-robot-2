/**
  * @file    adc_current.c
  * @brief   ADC电流采样 (移植自V3P, 适配STM32F407 HAL)
  *          PA3(IN3), PA6(IN6), PA4(IN4), PA7(IN7)
  *          DMA循环采集, 支持偏移校准和相电流计算
  */
#include "adc_current.h"
#include "stm32f4xx_hal.h"
#include "uart_comm.h"

/* DMA缓冲区: [0]=PA3(M1_Ia), [1]=PA6(M1_Ib), [2]=PA4(M2_Ia), [3]=PA7(M2_Ib) */
volatile uint16_t adc_dma_buf[ADC_CHANNELS] = {0};

/* 本模块私有句柄 (不与任何其他模块冲突) */
static ADC_HandleTypeDef  my_hadc;
static DMA_HandleTypeDef  my_hdma;
static uint8_t adc_started = 0;

uint8_t ADC_Is_Started(void)
{
    return adc_started;
}

/******************************************************************
 * ADC初始化 (4通道DMA循环采集)
 ******************************************************************/
void ADC_Current_Init(void)
{
    /* 防止重复初始化 */
    if (adc_started) return;

    /* ---- 时钟使能 ---- */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* ---- GPIO: PA3, PA4, PA6, PA7 模拟输入 ---- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ---- DMA2 Stream0 Channel0 ---- */
    my_hdma.Instance                 = DMA2_Stream0;
    my_hdma.Init.Channel             = DMA_CHANNEL_0;
    my_hdma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    my_hdma.Init.PeriphInc           = DMA_PINC_DISABLE;
    my_hdma.Init.MemInc              = DMA_MINC_ENABLE;
    my_hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    my_hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    my_hdma.Init.Mode                = DMA_CIRCULAR;
    my_hdma.Init.Priority            = DMA_PRIORITY_HIGH;
    my_hdma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&my_hdma);

    /* ---- ADC1 ---- */
    my_hadc.Instance                   = ADC1;
    my_hadc.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV6; /* 168/6=28MHz, <36MHz max */
    my_hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    my_hadc.Init.ScanConvMode          = ENABLE;
    my_hadc.Init.ContinuousConvMode    = ENABLE;
    my_hadc.Init.DiscontinuousConvMode = DISABLE;
    my_hadc.Init.NbrOfDiscConversion   = 0;
    my_hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    my_hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    my_hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    my_hadc.Init.NbrOfConversion       = ADC_CHANNELS;
    my_hadc.Init.DMAContinuousRequests  = ENABLE;
    my_hadc.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&my_hadc);

    __HAL_LINKDMA(&my_hadc, DMA_Handle, my_hdma);

    /* ---- 4个通道 ---- */
    ADC_ChannelConfTypeDef ch = {0};
    ch.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    ch.Channel = ADC_CHANNEL_3;  ch.Rank = 1;  HAL_ADC_ConfigChannel(&my_hadc, &ch); /* PA3 M1_Ia */
    ch.Channel = ADC_CHANNEL_6;  ch.Rank = 2;  HAL_ADC_ConfigChannel(&my_hadc, &ch); /* PA6 M1_Ib */
    ch.Channel = ADC_CHANNEL_4;  ch.Rank = 3;  HAL_ADC_ConfigChannel(&my_hadc, &ch); /* PA4 M2_Ia */
    ch.Channel = ADC_CHANNEL_7;  ch.Rank = 4;  HAL_ADC_ConfigChannel(&my_hadc, &ch); /* PA7 M2_Ib */

    /* ---- DMA中断 ---- */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    /* ---- ADC校准 (关键! STM32F407必须校准才能获得准确读数) ---- */
    /* ADC_CR2 bit2 = ADCAL (校准位) */
    HAL_ADC_Stop(&my_hadc);
    my_hadc.Instance->CR2 |= (1U << 2);   /* 启动校准 */
    while (my_hadc.Instance->CR2 & (1U << 2)) {}  /* 等待校准完成 */
    UART_Printf("ADC calibration done\r\n");

    /* ---- 启动 ---- */
    HAL_ADC_Start_DMA(&my_hadc, (uint32_t *)adc_dma_buf, ADC_CHANNELS);
    adc_started = 1;

    UART_Printf("ADC1 started, DMA buf[0-3]=%u %u %u %u\r\n",
        adc_dma_buf[0], adc_dma_buf[1], adc_dma_buf[2], adc_dma_buf[3]);
}

/******************************************************************
 * 偏移校准 (电机静止时调用, 多次采样取平均)
 *   每次采样等待1ms让DMA更新
 ******************************************************************/
static void DriftOffsets(Current_Sensor_t *sensor)
{
    uint16_t detect_rounds = 1000;
    float sum_ia = 0.0f;
    float sum_ib = 0.0f;

    /* 确定ADC通道索引: M1=[0,1], M2=[2,3] */
    int idx_a = (sensor->Sen_Num == 0) ? 0 : 2;
    int idx_b = idx_a + 1;

    /* 打印初始ADC原始值 */
    UART_Printf("Calib M%d: ADC raw [%u %u %u %u]\r\n",
        sensor->Sen_Num + 1,
        adc_dma_buf[0], adc_dma_buf[1],
        adc_dma_buf[2], adc_dma_buf[3]);

    /* 多次采样取平均 */
    for (int i = 0; i < detect_rounds; i++)
    {
        sum_ia += (float)adc_dma_buf[idx_a] * ADC_CONV;
        sum_ib += (float)adc_dma_buf[idx_b] * ADC_CONV;
        HAL_Delay(1);
    }

    sensor->offset_ia = sum_ia / detect_rounds;
    sensor->offset_ib = sum_ib / detect_rounds;

    /* 打印详细的校准结果 */
    UART_Printf("Calib M%d: avg_ia=%.4fV(%d) avg_ib=%.4fV(%d) gain=%.0f\r\n",
        sensor->Sen_Num + 1,
        sensor->offset_ia, (int)(sensor->offset_ia / ADC_CONV),
        sensor->offset_ib, (int)(sensor->offset_ib / ADC_CONV),
        sensor->gain_sign);
}

/******************************************************************
 * 电流传感器初始化 (校准偏移)
 *   注意: 调用前必须:
 *   1. 停止PWM输出 (setPhaseVoltage(0,0,0))
 *   2. 等待足够时间让电流降为0
 ******************************************************************/
void CurrSense_Init(Current_Sensor_t *sensor)
{
    sensor->I_a = 0.0f;
    sensor->I_b = 0.0f;
    sensor->offset_ia = 0.0f;
    sensor->offset_ib = 0.0f;
    /* 默认增益符号: M1取反(V3P), M2同相 */
    if (sensor->gain_sign == 0.0f)
        sensor->gain_sign = (sensor->Sen_Num == 0) ? -1.0f : 1.0f;

    DriftOffsets(sensor);
}

/******************************************************************
 * ADC原始值IIR低通滤波 (抑制PWM开关噪声)
 *   alpha越小滤波越强, 但延迟越大
 *   alpha=0.2 ≈ 5次平均效果, 延迟约2个采样周期
 ******************************************************************/
static float adc_filtered[4] = {0};
static uint8_t adc_filter_inited = 0;
#define ADC_FILTER_ALPHA  0.2f

static void adc_filter_update(void)
{
    if (!adc_filter_inited)
    {
        /* 首次用原始值初始化 */
        for (int i = 0; i < 4; i++)
            adc_filtered[i] = (float)adc_dma_buf[i];
        adc_filter_inited = 1;
    }
    else
    {
        /* IIR低通: y = alpha*x + (1-alpha)*y */
        for (int i = 0; i < 4; i++)
            adc_filtered[i] += ADC_FILTER_ALPHA * ((float)adc_dma_buf[i] - adc_filtered[i]);
    }
}

void ADC_Filter_Reset(void)
{
    adc_filter_inited = 0;
    for (int i = 0; i < 4; i++)
        adc_filtered[i] = 0;
}

/******************************************************************
 * 获取相电流 (ADC原始值 → 滤波 → 电压 → 电流)
 *   gain = 1 / R_shunt / Amp_gain = 2.0 A/V
 ******************************************************************/
void GetPhaseCurrent(Current_Sensor_t *sensor)
{
    /* 更新ADC滤波值 */
    adc_filter_update();

    /* 确定ADC通道索引 */
    int idx_a = (sensor->Sen_Num == 0) ? 0 : 2;
    int idx_b = idx_a + 1;

    /* 滤波后的ADC → 电压 */
    float voltage_a = adc_filtered[idx_a] * ADC_CONV;
    float voltage_b = adc_filtered[idx_b] * ADC_CONV;

    /* 电压 → 电流 (减偏移, 乘增益) */
    float vlots_to_amps = 1.0f / SHUNT_RESISTOR / AMP_GAIN;

    sensor->I_a = (voltage_a - sensor->offset_ia) * (sensor->gain_sign * vlots_to_amps);
    sensor->I_b = (voltage_b - sensor->offset_ib) * (sensor->gain_sign * vlots_to_amps);
}

/******************************************************************
 * DMA中断处理
 ******************************************************************/
void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&my_hdma);
}

/******************************************************************
 * ADC MspInit (HAL回调, 自动调用)
 ******************************************************************/
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_ENABLE();
    }
}
