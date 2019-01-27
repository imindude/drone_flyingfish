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

#include "soc_gpio.h"

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    GPIO_TypeDef    *gpio_;
    uint16_t        pins_;
}
LocalData;

/* ****************************************************************************************************************** */

static LocalData    gpio_data[_GpioPin_N] = {

        [_GpioPin_1] = {

                .gpio_ = GPIOB,
                .pins_ = GPIO_PIN_9
        },
        [_GpioPin_2] = {

                .gpio_ = GPIOC,
                .pins_ = GPIO_PIN_2
        },
        [_GpioPin_3] = {

                .gpio_ = GPIOC,
                .pins_ = GPIO_PIN_3
        }
};

/* ****************************************************************************************************************** */

GpioHandle soc_gpio_init(GpioPin pin)
{
    GPIO_InitTypeDef    gpio_init;

    if (pin < _GpioPin_N)
    {
        gpio_init.Pin       = gpio_data[pin].pins_;
        gpio_init.Mode      = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull      = GPIO_NOPULL;
        gpio_init.Speed     = GPIO_SPEED_FREQ_MEDIUM;
        gpio_init.Alternate = 0;

        HAL_GPIO_Init(gpio_data[pin].gpio_, &gpio_init);

        return (GpioHandle)(&gpio_data[pin]);
    }

    return NULL;
}

void soc_gpio_term(GpioHandle h)
{
    LocalData   *this = (LocalData*)h;

    HAL_GPIO_DeInit(this->gpio_, this->pins_);
}

void soc_gpio_set_high(GpioHandle h)
{
    LocalData   *this = (LocalData*)h;

    HAL_GPIO_WritePin(this->gpio_, this->pins_, GPIO_PIN_SET);
}

void soc_gpio_set_low(GpioHandle h)
{
    LocalData   *this = (LocalData*)h;

    HAL_GPIO_WritePin(this->gpio_, this->pins_, GPIO_PIN_RESET);
}

/* end of file ****************************************************************************************************** */
