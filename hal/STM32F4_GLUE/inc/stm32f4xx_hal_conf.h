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

#ifndef __STM32F4XX_HAL_CONF_H__
#define __STM32F4XX_HAL_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ****************************************************************************************************************** */

#if !defined HSE_VALUE
# define HSE_VALUE                  ((uint32_t)25000000U)
#endif

#if !defined HSE_STARTUP_TIMEOUT
# define HSE_STARTUP_TIMEOUT        ((uint32_t)100U)
#endif

#if !defined HSI_VALUE
# define HSI_VALUE                  ((uint32_t)16000000U)
#endif

#if !defined LSI_VALUE
# define LSI_VALUE                  ((uint32_t)32000U)
#endif

#if !defined LSE_VALUE
# define LSE_VALUE                  ((uint32_t)32768U)
#endif

#if !defined LSE_STARTUP_TIMEOUT
# define LSE_STARTUP_TIMEOUT        ((uint32_t)5000U)
#endif

#if !defined EXTERNAL_CLOCK_VALUE
# define EXTERNAL_CLOCK_VALUE       ((uint32_t)12288000U)
#endif

/**
 * @brief This is the HAL system configuration section
 */
#define  VDD_VALUE                  ((uint32_t)3300U)
#define  TICK_INT_PRIORITY          ((uint32_t)0U)
#define  USE_RTOS                   0U
#define  PREFETCH_ENABLE            1U
#define  INSTRUCTION_CACHE_ENABLE   1U
#define  DATA_CACHE_ENABLE          1U

#define USE_SPI_CRC                 0U
#define assert_param(expr)          ((void)0U)

/* ****************************************************************************************************************** */

#define HAL_CORTEX_MODULE_ENABLED
#include "stm32f4xx_hal_cortex.h"

#define HAL_DMA_MODULE_ENABLED
#include "stm32f4xx_hal_dma.h"

#define HAL_FLASH_MODULE_ENABLED
#include "stm32f4xx_hal_flash.h"

#define HAL_GPIO_MODULE_ENABLED
#include "stm32f4xx_hal_gpio.h"

#define HAL_PCD_MODULE_ENABLED
#include "stm32f4xx_hal_pcd.h"

#define HAL_PWR_MODULE_ENABLED
#include "stm32f4xx_hal_pwr.h"

#define HAL_RCC_MODULE_ENABLED
#include "stm32f4xx_hal_rcc.h"

#define HAL_SPI_MODULE_ENABLED
#include "stm32f4xx_hal_spi.h"

#define HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"

#define HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"

/* ****************************************************************************************************************** */

extern uint32_t sys_get_millis(void);
extern uint32_t sys_get_micros(void);
extern void     sys_delay_millis(uint32_t ms);
extern void     sys_delay_micros(uint32_t us);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
 }
#endif

#endif  // __STM32F4XX_HAL_CONF_H__

/* end of file ****************************************************************************************************** */
