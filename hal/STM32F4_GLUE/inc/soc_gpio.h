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

#ifndef __SOC_GPIO_H__
#define __SOC_GPIO_H__

/* ****************************************************************************************************************** */

#include <stdint.h>
#include "stm32f4xx.h"

/* ****************************************************************************************************************** */

typedef enum _GpioPin
{
    _GpioPin_1,     // LED
    _GpioPin_2,     // BMP280 nCS
    _GpioPin_3,     // MPU9255 nCS
    _GpioPin_N
}
GpioPin;

typedef void    *GpioHandle;

/* ****************************************************************************************************************** */

extern GpioHandle   soc_gpio_init(GpioPin pin);
extern void         soc_gpio_term(GpioHandle h);
extern void         soc_gpio_set_high(GpioHandle h);
extern void         soc_gpio_set_low(GpioHandle h);

/* ****************************************************************************************************************** */

#endif /* __SOC_GPIO_H__ */

/* end of file ****************************************************************************************************** */