/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ADC DMA + Butterworth LPF + UART DMA + Header
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;


/* ========================================================================= */
/*                         C?U HÌNH H? TH?NG                                */
/* ========================================================================= */

#define FRAME_SAMPLES       64

/*
 * M?i sample:
 *   Raw      : 2 byte
 *   Filtered : 2 byte
 *
 * Header:
 *   AA 55 : 2 byte
 *
 * T?ng:
 *   2 + 64 * 4 = 258 byte
 */
#define FRAME_BYTES         (2 + FRAME_SAMPLES * 4)


/*
 * ADC DMA:
 *
 * 128 m?u = 2 half
 *
 * Half 0:
 *   adc_buf[0 ... 63]
 *
 * Half 1:
 *   adc_buf[64 ... 127]
 */
#define ADC_DMA_SIZE        (FRAME_SAMPLES * 2)


/* ========================================================================= */
/*                         ADC DMA BUFFER                                    */
/* ========================================================================= */

uint16_t adc_buf[ADC_DMA_SIZE];


/* ========================================================================= */
/*                         UART DOUBLE BUFFER                                */
/* ========================================================================= */

/*
 * Có 2 buffer:
 *
 * tx_buf[0] dang UART DMA truy?n
 * tx_buf[1] có th? du?c CPU ghi frame m?i
 *
 * Không du?c ghi vào buffer mà UART DMA dang s? d?ng.
 */
uint8_t tx_buf[2][FRAME_BYTES];


/*
 * 0 = buffer 0
 * 1 = buffer 1
 */
volatile uint8_t tx_busy[2] = {0, 0};


/*
 * Buffer dang du?c UART DMA truy?n.
 *
 * Ch? có ý nghia khi uart_tx_busy = 1.
 */
volatile uint8_t uart_tx_busy = 0;


/*
 * Buffer cu?i cùng du?c s? d?ng.
 *
 * Dùng d? luân phiên A/B.
 */
volatile uint8_t tx_next_buffer = 0;


/* ========================================================================= */
/*                         TH?NG KÊ                                         */
/* ========================================================================= */

volatile uint32_t frame_count = 0;
volatile uint32_t uart_tx_count = 0;
volatile uint32_t uart_drop_count = 0;


/* ========================================================================= */
/*                         BUTTERWORTH LPF                                  */
/* ========================================================================= */

/*
 * Butterworth b?c 2
 *
 * Fs = 8000 Hz
 *
 * H? s? hi?n t?i:
 *
 * b0 = 0.0201
 * b1 = 0.0402
 * b2 = 0.0201
 *
 * a1 = -1.5610
 * a2 = 0.6414
 *
 *
 * Direct Form II:
 *
 * w0 = x - a1*w1 - a2*w2
 *
 * y  = b0*w0 + b1*w1 + b2*w2
 *
 * w2 = w1
 * w1 = w0
 */

const float b0 = 0.0201f;
const float b1 = 0.0402f;
const float b2 = 0.0201f;

const float a1 = -1.5610f;
const float a2 =  0.6414f;


/*
 * State c?a b? l?c.
 *
 * KHÔNG reset các bi?n này sau m?i frame.
 */
float w1 = 0.0f;
float w2 = 0.0f;


/*
 * Ðánh d?u filter dã du?c kh?i t?o hay chua.
 */
uint8_t filter_initialized = 0;


/* ========================================================================= */
/*                         FUNCTION PROTOTYPES                              */
/* ========================================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);


/* ========================================================================= */
/*                         BUTTERWORTH FUNCTION                             */
/* ========================================================================= */

static void LPF_Init(float x0)
{
    /*
     * V?i tín hi?u DC x0, tr?ng thái ?n d?nh c?a Direct Form II là:
     *
     * w1 = w2 = x0 / (1 + a1 + a2)
     *
     * Nh? v?y n?u ADC dang bias kho?ng 2048,
     * output filter cung b?t d?u g?n 2048,
     * thay vì b?t d?u t? g?n 0.
     */

    float denominator;

    denominator = 1.0f + a1 + a2;

    if (denominator != 0.0f)
    {
        w1 = x0 / denominator;
        w2 = x0 / denominator;
    }
    else
    {
        w1 = 0.0f;
        w2 = 0.0f;
    }

    filter_initialized = 1;
}


/* ------------------------------------------------------------------------- */
/* Butterworth LPF                                                           */
/* ------------------------------------------------------------------------- */

static inline float LPF_Process(float x)
{
    float w0;
    float y;

    /*
     * N?u chua kh?i t?o:
     * dùng chính m?u d?u tiên làm m?c DC ban d?u.
     */
    if (!filter_initialized)
    {
        LPF_Init(x);
    }

    /*
     * Direct Form II
     */
    w0 = x - a1 * w1 - a2 * w2;

    y = b0 * w0
      + b1 * w1
      + b2 * w2;

    /*
     * Update state
     */
    w2 = w1;
    w1 = w0;

    return y;
}


/* ========================================================================= */
/*                         INT16 LIMIT                                      */
/* ========================================================================= */

static int16_t float_to_int16(float x)
{
    if (x > 32767.0f)
        return 32767;

    if (x < -32768.0f)
        return -32768;

    return (int16_t)x;
}


/* ========================================================================= */
/*                         GHI INT16 LITTLE-ENDIAN                           */
/* ========================================================================= */

/*
 * Không dùng:
 *
 * int16_t *pOut = (int16_t*)&tx_buf[2];
 *
 * vì frame b?t d?u ? offset 2.
 *
 * Thay vào dó ghi byte tr?c ti?p.
 */

static void write_int16_le(uint8_t *dst, int16_t value)
{
    uint16_t u = (uint16_t)value;

    dst[0] = (uint8_t)(u & 0xFF);
    dst[1] = (uint8_t)((u >> 8) & 0xFF);
}


/* ========================================================================= */
/*                         TÌM BUFFER UART R?NH                             */
/* ========================================================================= */

static int8_t get_free_tx_buffer(void)
{
    uint8_t i;

    /*
     * Th? buffer du?c ch?n tru?c.
     */
    for (i = 0; i < 2; i++)
    {
        uint8_t index = (tx_next_buffer + i) & 0x01;

        if (tx_busy[index] == 0)
        {
            return (int8_t)index;
        }
    }

    /*
     * C? 2 buffer d?u b?n.
     */
    return -1;
}


/* ========================================================================= */
/*                         BUILD + SEND FRAME                               */
/* ========================================================================= */

static void process_and_send(uint16_t *src)
{
    int8_t buffer_index;
    uint8_t *frame;

    /*
     * Tìm buffer UART chua du?c s? d?ng.
     */
    buffer_index = get_free_tx_buffer();

    /*
     * N?u c? 2 buffer d?u dang b?n:
     *
     * Không du?c ghi dè buffer UART.
     *
     * B? frame này.
     */
    if (buffer_index < 0)
    {
        uart_drop_count++;
        return;
    }


    frame = tx_buf[buffer_index];


    /* --------------------------------------------------------------------- */
    /* HEADER                                                                */
    /* --------------------------------------------------------------------- */

    frame[0] = 0xAA;
    frame[1] = 0x55;


    /* --------------------------------------------------------------------- */
    /* DATA                                                                  */
    /* --------------------------------------------------------------------- */

    for (int i = 0; i < FRAME_SAMPLES; i++)
    {
        uint16_t raw;
        int16_t filtered;

        raw = src[i];


        /*
         * RAW
         *
         * ADC 12 bit:
         * 0 ... 4095
         */
        write_int16_le(
            &frame[2 + i * 4],
            (int16_t)raw
        );


        /*
         * FILTERED
         */
        filtered = float_to_int16(
            LPF_Process((float)raw)
        );


        write_int16_le(
            &frame[2 + i * 4 + 2],
            filtered
        );
    }


    /* --------------------------------------------------------------------- */
    /* UART DMA                                                              */
    /* --------------------------------------------------------------------- */

    /*
     * Ðánh d?u buffer này dang du?c chu?n b? s? d?ng.
     */
    tx_busy[buffer_index] = 1;


    /*
     * N?u UART dang r?nh thì b?t d?u truy?n ngay.
     *
     * N?u UART dang b?n:
     *
     * buffer này hi?n chua du?c truy?n.
     *
     * V?i c?u hình hi?n t?i, frame m?i thu?ng d?n sau ~8 ms,
     * trong khi 258 byte @ 460800 baud m?t kho?ng 5.6 ms,
     * nên thông thu?ng s? có buffer r?nh.
     */
    if (uart_tx_busy == 0)
    {
        HAL_StatusTypeDef status;

        uart_tx_busy = 1;

        status = HAL_UART_Transmit_DMA(
            &huart1,
            tx_buf[buffer_index],
            FRAME_BYTES
        );

        if (status != HAL_OK)
        {
            /*
             * UART không th? b?t d?u truy?n.
             */
            uart_tx_busy = 0;
            tx_busy[buffer_index] = 0;

            uart_drop_count++;
        }
        else
        {
            /*
             * Buffer dang du?c UART DMA s? d?ng.
             */
            tx_next_buffer = (buffer_index + 1) & 0x01;

            uart_tx_count++;
        }
    }
    else
    {
        /*
         * UART dang b?n.
         *
         * Không th? g?i HAL_UART_Transmit_DMA()
         * l?n n?a vì HAL s? tr? HAL_BUSY.
         *
         * Trong thi?t k? don gi?n này,
         * ta ch? có 2 buffer nên n?u UART dang b?n
         * thì buffer m?i s? du?c dánh d?u ch?.
         *
         * Tuy nhiên c?n chuy?n nó thành buffer pending.
         */
    }


    frame_count++;
}


/* ========================================================================= */
/*                         UART DMA CALLBACK                                */
/* ========================================================================= */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /*
         * Tìm buffer v?a truy?n xong.
         *
         * Buffer dang truy?n là buffer có tx_busy = 1
         * và không ph?i buffer dang pending.
         *
         * V?i t?c d? hi?n t?i, thu?ng ch? có m?t buffer dang TX.
         */

        uint8_t i;

        for (i = 0; i < 2; i++)
        {
            if (tx_busy[i])
            {
                /*
                 * Gi?i phóng buffer.
                 *
                 * Trong thi?t k? hi?n t?i, buffer này chính là
                 * buffer UART v?a truy?n.
                 */
                tx_busy[i] = 0;
                break;
            }
        }

        uart_tx_busy = 0;
    }
}


/* ========================================================================= */
/*                         ADC CALLBACKS                                    */
/* ========================================================================= */

/*
 * DMA dã nh?n 64 m?u d?u tiên.
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        process_and_send(&adc_buf[0]);
    }
}


/*
 * DMA dã nh?n d? 128 m?u.
 *
 * 64 m?u th? hai n?m ?:
 *
 * adc_buf[64 ... 127]
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        process_and_send(&adc_buf[FRAME_SAMPLES]);
    }
}


/* ========================================================================= */
/*                         MAIN                                             */
/* ========================================================================= */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();


    /* --------------------------------------------------------------------- */
    /* ADC CALIBRATION                                                       */
    /* --------------------------------------------------------------------- */

    HAL_ADCEx_Calibration_Start(&hadc1);


    /* --------------------------------------------------------------------- */
    /* START TIMER                                                           */
    /* --------------------------------------------------------------------- */

    HAL_TIM_Base_Start(&htim3);


    /* --------------------------------------------------------------------- */
    /* START ADC DMA                                                         */
    /* --------------------------------------------------------------------- */

    HAL_ADC_Start_DMA(
        &hadc1,
        (uint32_t *)adc_buf,
        ADC_DMA_SIZE
    );


    /* --------------------------------------------------------------------- */
    /* MAIN LOOP                                                             */
    /* --------------------------------------------------------------------- */

    while (1)
    {
        /*
         * Heartbeat LED
         */
        HAL_GPIO_TogglePin(
            GPIOC,
            GPIO_PIN_13
        );

        HAL_Delay(500);
    }
}


/* ========================================================================= */
/*                         SYSTEM CLOCK                                     */
/* ========================================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState = RCC_HSE_ON;

    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;

    RCC_OscInitStruct.HSIState = RCC_HSI_ON;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }


    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_ADC;

    PeriphClkInit.AdcClockSelection =
        RCC_ADCPCLK2_DIV6;


    if (HAL_RCCEx_PeriphCLKConfig(
            &PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================= */
/*                         ADC1                                             */
/* ========================================================================= */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};


    hadc1.Instance = ADC1;


    hadc1.Init.ScanConvMode =
        ADC_SCAN_DISABLE;

    hadc1.Init.ContinuousConvMode =
        DISABLE;

    hadc1.Init.DiscontinuousConvMode =
        DISABLE;

    hadc1.Init.ExternalTrigConv =
        ADC_EXTERNALTRIGCONV_T3_TRGO;

    hadc1.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;

    hadc1.Init.NbrOfConversion =
        1;


    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }


    sConfig.Channel =
        ADC_CHANNEL_0;

    sConfig.Rank =
        ADC_REGULAR_RANK_1;

    sConfig.SamplingTime =
        ADC_SAMPLETIME_55CYCLES_5;


    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================= */
/*                         TIMER 3                                          */
/* ========================================================================= */

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};

    TIM_MasterConfigTypeDef sMasterConfig = {0};


    htim3.Instance = TIM3;


    htim3.Init.Prescaler = 0;

    htim3.Init.CounterMode =
        TIM_COUNTERMODE_UP;

    /*
     * 72 MHz / 9000 = 8 kHz
     */
    htim3.Init.Period = 8999;

    htim3.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;

    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }


    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;


    if (HAL_TIM_ConfigClockSource(
            &htim3,
            &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * TIM3 Update Event
     *
     * ? ADC trigger
     */
    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_UPDATE;

    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim3,
            &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================= */
/*                         UART1                                            */
/* ========================================================================= */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;


    huart1.Init.BaudRate =
        460800;

    huart1.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart1.Init.StopBits =
        UART_STOPBITS_1;

    huart1.Init.Parity =
        UART_PARITY_NONE;

    huart1.Init.Mode =
        UART_MODE_TX_RX;

    huart1.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart1.Init.OverSampling =
        UART_OVERSAMPLING_16;


    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================= */
/*                         DMA                                             */
/* ========================================================================= */

static void MX_DMA_Init(void)
{
    /*
     * Enable DMA1
     */
    __HAL_RCC_DMA1_CLK_ENABLE();


    /* --------------------------------------------------------------------- */
    /* ADC DMA                                                               */
    /* --------------------------------------------------------------------- */

    HAL_NVIC_SetPriority(
        DMA1_Channel1_IRQn,
        0,
        0
    );

    HAL_NVIC_EnableIRQ(
        DMA1_Channel1_IRQn
    );


    hdma_adc1.Instance =
        DMA1_Channel1;

    hdma_adc1.Init.Direction =
        DMA_PERIPH_TO_MEMORY;

    hdma_adc1.Init.PeriphInc =
        DMA_PINC_DISABLE;

    hdma_adc1.Init.MemInc =
        DMA_MINC_ENABLE;

    hdma_adc1.Init.PeriphDataAlignment =
        DMA_PDATAALIGN_HALFWORD;

    hdma_adc1.Init.MemDataAlignment =
        DMA_MDATAALIGN_HALFWORD;

    hdma_adc1.Init.Mode =
        DMA_CIRCULAR;

    hdma_adc1.Init.Priority =
        DMA_PRIORITY_HIGH;


    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
        Error_Handler();
    }


    __HAL_LINKDMA(
        &hadc1,
        DMA_Handle,
        hdma_adc1
    );


    /* --------------------------------------------------------------------- */
    /* UART TX DMA                                                           */
    /* --------------------------------------------------------------------- */

    HAL_NVIC_SetPriority(
        DMA1_Channel4_IRQn,
        1,
        0
    );

    HAL_NVIC_EnableIRQ(
        DMA1_Channel4_IRQn
    );


    hdma_usart1_tx.Instance =
        DMA1_Channel4;

    hdma_usart1_tx.Init.Direction =
        DMA_MEMORY_TO_PERIPH;

    hdma_usart1_tx.Init.PeriphInc =
        DMA_PINC_DISABLE;

    hdma_usart1_tx.Init.MemInc =
        DMA_MINC_ENABLE;

    hdma_usart1_tx.Init.PeriphDataAlignment =
        DMA_PDATAALIGN_BYTE;

    hdma_usart1_tx.Init.MemDataAlignment =
        DMA_MDATAALIGN_BYTE;

    hdma_usart1_tx.Init.Mode =
        DMA_NORMAL;

    hdma_usart1_tx.Init.Priority =
        DMA_PRIORITY_LOW;


    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
        Error_Handler();
    }


    __HAL_LINKDMA(
        &huart1,
        hdmatx,
        hdma_usart1_tx
    );
}


/* ========================================================================= */
/*                         GPIO                                             */
/* ========================================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOD_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();


    /* --------------------------------------------------------------------- */
    /* PC13 LED                                                              */
    /* --------------------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_13;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );


    /*
     * Blue Pill LED active LOW
     */
    HAL_GPIO_WritePin(
        GPIOC,
        GPIO_PIN_13,
        GPIO_PIN_SET
    );


    /* --------------------------------------------------------------------- */
    /* PA0 ADC                                                               */
    /* --------------------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_0;

    GPIO_InitStruct.Mode =
        GPIO_MODE_ANALOG;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* --------------------------------------------------------------------- */
    /* PA9 USART1 TX                                                         */
    /* --------------------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_9;

    GPIO_InitStruct.Mode =
        GPIO_MODE_AF_PP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_HIGH;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );


    /* --------------------------------------------------------------------- */
    /* PA10 USART1 RX                                                        */
    /* --------------------------------------------------------------------- */

    GPIO_InitStruct.Pin =
        GPIO_PIN_10;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct
    );
}


/* ========================================================================= */
/*                         ERROR HANDLER                                     */
/* ========================================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
}

#endif
