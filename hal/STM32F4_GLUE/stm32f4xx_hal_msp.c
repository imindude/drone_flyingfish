/**
 * *********************************************************************************************************************
 *  __     __  __
 * |  |   |__ |__
 * |__| X |   |     FlyingFish
 *
 * *********************************************************************************************************************
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "stm32f4xx.h"

/* ****************************************************************************************************************** */

SPI_HandleTypeDef   hspi3 = {

        .Instance = SPI3,
        .Init.Mode              = SPI_MODE_MASTER,
        .Init.Direction         = SPI_DIRECTION_2LINES,
        .Init.DataSize          = SPI_DATASIZE_8BIT,
        .Init.CLKPolarity       = SPI_POLARITY_HIGH,
        .Init.CLKPhase          = SPI_PHASE_2EDGE,
        .Init.NSS               = SPI_NSS_SOFT,
        .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
        .Init.FirstBit          = SPI_FIRSTBIT_MSB,
        .Init.TIMode            = SPI_TIMODE_DISABLE,
        .Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE,
        .Init.CRCPolynomial     = 10
};

DMA_HandleTypeDef hdma_spi3_tx = {

        .Instance = DMA1_Stream7,
        .Init.Channel             = DMA_CHANNEL_0,
        .Init.Direction           = DMA_MEMORY_TO_PERIPH,
        .Init.PeriphInc           = DMA_PINC_DISABLE,
        .Init.MemInc              = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
        .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
        .Init.Mode                = DMA_NORMAL,
        .Init.Priority            = DMA_PRIORITY_LOW,
        .Init.FIFOMode            = DMA_FIFOMODE_DISABLE
};

DMA_HandleTypeDef hdma_spi3_rx = {

        .Instance = DMA1_Stream0,
        .Init.Channel             = DMA_CHANNEL_0,
        .Init.Direction           = DMA_PERIPH_TO_MEMORY,
        .Init.PeriphInc           = DMA_PINC_DISABLE,
        .Init.MemInc              = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
        .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
        .Init.Mode                = DMA_NORMAL,
        .Init.Priority            = DMA_PRIORITY_LOW,
        .Init.FIFOMode            = DMA_FIFOMODE_DISABLE
};

TIM_HandleTypeDef htim2 = {

        .Instance = TIM2,
        .Init.Prescaler     = 0,
        .Init.CounterMode   = TIM_COUNTERMODE_UP,
        .Init.Period        = 0,
        .Init.ClockDivision = TIM_CLOCKDIVISION_DIV1
};

TIM_HandleTypeDef htim3 = {

        .Instance = TIM3,
        .Init.Prescaler     = 0,
        .Init.CounterMode   = TIM_COUNTERMODE_UP,
        .Init.Period        = 0,
        .Init.ClockDivision = TIM_CLOCKDIVISION_DIV1,
};

UART_HandleTypeDef huart1 = {

        .Instance = USART1,
        .Init.BaudRate     = 115200,
        .Init.WordLength   = UART_WORDLENGTH_8B,
        .Init.StopBits     = UART_STOPBITS_1,
        .Init.Parity       = UART_PARITY_NONE,
        .Init.Mode         = UART_MODE_TX_RX,
        .Init.HwFlowCtl    = UART_HWCONTROL_NONE,
        .Init.OverSampling = UART_OVERSAMPLING_16
};

DMA_HandleTypeDef hdma_usart1_tx = {

        .Instance = DMA2_Stream7,
        .Init.Channel             = DMA_CHANNEL_4,
        .Init.Direction           = DMA_MEMORY_TO_PERIPH,
        .Init.PeriphInc           = DMA_PINC_DISABLE,
        .Init.MemInc              = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
        .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
        .Init.Mode                = DMA_NORMAL,
        .Init.Priority            = DMA_PRIORITY_LOW,
        .Init.FIFOMode            = DMA_FIFOMODE_DISABLE
};

DMA_HandleTypeDef hdma_usart1_rx = {

        .Instance = DMA2_Stream2,
        .Init.Channel             = DMA_CHANNEL_4,
        .Init.Direction           = DMA_PERIPH_TO_MEMORY,
        .Init.PeriphInc           = DMA_PINC_DISABLE,
        .Init.MemInc              = DMA_MINC_ENABLE,
        .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
        .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
        .Init.Mode                = DMA_NORMAL,
        .Init.Priority            = DMA_PRIORITY_LOW,
        .Init.FIFOMode            = DMA_FIFOMODE_DISABLE
};

UART_HandleTypeDef huart6 = {

        .Instance = USART6,
        .Init.BaudRate     = 115200,
        .Init.WordLength   = UART_WORDLENGTH_8B,
        .Init.StopBits     = UART_STOPBITS_1,
        .Init.Parity       = UART_PARITY_NONE,
        .Init.Mode         = UART_MODE_TX_RX,
        .Init.HwFlowCtl    = UART_HWCONTROL_NONE,
        .Init.OverSampling = UART_OVERSAMPLING_16
};

static volatile uint32_t    micros_ticks;       // clock count every milli-second

/* ****************************************************************************************************************** */

static void hal_InitSysClock(void)
{
    RCC_OscInitTypeDef  RCC_OscInitStruct;
    RCC_ClkInitTypeDef  RCC_ClkInitStruct;

    /**
     * Configure the main internal regulator output voltage
     */

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**
     * Initializes the CPU, AHB and APB busses clocks
     */

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;

    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /**
     * Initializes the CPU, AHB and APB busses clocks
     */

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

    /**
     * Configure the Systick interrupt time
     */

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**
     * Configure the Systick
     */

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    micros_ticks = HAL_RCC_GetSysClockFreq() / 1000000;
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    hal_InitSysClock();
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (hspi->Instance == SPI3) {

        __HAL_RCC_SPI3_CLK_ENABLE();

        /**
         * SPI3 GPIO Configuration
         * PB3     ------> SPI3_SCK
         * PB4     ------> SPI3_MISO
         * PB5     ------> SPI3_MOSI
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;

        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* SPI3 DMA Init */

        HAL_DMA_Init(&hdma_spi3_tx);
        HAL_DMA_Init(&hdma_spi3_rx);

        __HAL_LINKDMA(hspi, hdmatx, hdma_spi3_tx);
        __HAL_LINKDMA(hspi, hdmarx, hdma_spi3_rx);

        HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    }
}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (htim_oc->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();

        /**
         * TIM2 GPIO Configuration
         * PA0-WKUP     ------> TIM2_CH1
         * PA1     ------> TIM2_CH2
         * PA2     ------> TIM2_CH3
         * PA3     ------> TIM2_CH4
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (htim_ic->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();

        /**
         * TIM3 GPIO Configuration
         * PC7     ------> TIM3_CH2
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;

        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();

        /**
         * USART1 GPIO Configuration
         * PA9     ------> USART1_TX
         * PA10     ------> USART1_RX
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 DMA Init */

        HAL_DMA_Init(&hdma_usart1_tx);
        HAL_DMA_Init(&hdma_usart1_rx);

        __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

        HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    }
    else if (huart->Instance == USART6)
    {
        __HAL_RCC_USART6_CLK_ENABLE();

        /**
         * USART6 GPIO Configuration
         * PC6     ------> USART6_TX
         * PC7     ------> USART6_RX
         */

        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;

        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}

uint32_t sys_get_millis(void)
{
    return HAL_GetTick();
}

uint32_t sys_get_micros(void)
{
    volatile uint32_t   now_ticks = SysTick->VAL;  	// down count
    return HAL_GetTick() * 1000 + (1000 - (now_ticks / micros_ticks));
}

void sys_delay_millis(uint32_t ms)
{
    HAL_Delay(ms);
}

void sys_delay_micros(uint32_t us)
{
    volatile uint32_t   target_us = sys_get_micros() + us;

    while (sys_get_micros() < target_us)
        __asm("nop\r\n");
}

/* end of file ****************************************************************************************************** */
