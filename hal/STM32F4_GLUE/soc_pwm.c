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

#include "soc_pwm.h"

/* ****************************************************************************************************************** */

extern TIM_HandleTypeDef    htim2;
extern TIM_HandleTypeDef    htim3;

#define PWM_MAX_PERIOD      0xFFFF

/* ****************************************************************************************************************** */

typedef struct _LocalDataIn
{
    TIM_HandleTypeDef   *pwm_;
    uint32_t            channel_;
    uint32_t            last_value_;
    PwmInCallback       callback_;
    void                *param_;
}
LocalDataIn;

/* ****************************************************************************************************************** */

static LocalDataIn  pwmin_data[_PwmInPin_N] = {

        [_PwmInPin_1] = {

                .pwm_        = &htim3,
                .channel_    = TIM_CHANNEL_2,
                .last_value_ = 0,
                .callback_   = NULL,
                .param_      = NULL
        }
};

/* ****************************************************************************************************************** */

PwmInHandle soc_pwmin_init(PwmInPin pin, PwmInCallback cbf, void *param)
{
    if (pin < _PwmInPin_N)
    {
        if (pwmin_data[pin].pwm_->State == HAL_TIM_STATE_RESET)
        {
            TIM_MasterConfigTypeDef     master_cfg;

            // TIM3 is on the APB1 - 84MHz
            pwmin_data[pin].pwm_->Init.Prescaler = HAL_RCC_GetPCLK1Freq() * 2 / 1000000;    // 1us per one period
            pwmin_data[pin].pwm_->Init.Period    = PWM_MAX_PERIOD;

            HAL_TIM_OC_Init(pwmin_data[pin].pwm_);

            master_cfg.MasterOutputTrigger = TIM_TRGO_RESET;
            master_cfg.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

            HAL_TIMEx_MasterConfigSynchronization(pwmin_data[pin].pwm_, &master_cfg);
        }

        TIM_IC_InitTypeDef  init_param;

        init_param.ICPolarity  = TIM_ICPOLARITY_RISING;
        init_param.ICSelection = TIM_ICSELECTION_DIRECTTI;
        init_param.ICPrescaler = TIM_ICPSC_DIV1;
        init_param.ICFilter    = 0;

        HAL_TIM_IC_ConfigChannel(pwmin_data[pin].pwm_, &init_param, pwmin_data[pin].channel_);
        HAL_TIM_IC_Start_IT(pwmin_data[pin].pwm_, pwmin_data[pin].channel_);

        pwmin_data[pin].callback_ = cbf;
        pwmin_data[pin].param_    = param;

        return (PwmInHandle)(&pwmin_data[pin]);
    }

    return NULL;
}

/* ****************************************************************************************************************** */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    for (int8_t i = 0; i < _PwmInPin_N; i++)
    {
        if (pwmin_data[i].pwm_ == htim)
        {
            LocalDataIn *this = &pwmin_data[i];
            uint32_t    now_value = HAL_TIM_ReadCapturedValue(htim, this->channel_);
            uint32_t    duty_us;

            if (now_value >= this->last_value_)
                duty_us = now_value - this->last_value_;
            else
                duty_us = PWM_MAX_PERIOD - this->last_value_ + now_value;

            if (this->callback_)
                this->callback_(duty_us, this->param_);

            break;
        }
    }
}

/* ****************************************************************************************************************** */

typedef struct _LocalDataOut
{
    TIM_HandleTypeDef   *pwm_;
    uint32_t            channel_;
    int16_t             us_tick_;
}
LocalDataOut;

/* ****************************************************************************************************************** */

static LocalDataOut pwmout_data[_PwmOutPin_N] = {

        [_PwmOutPin_1] = {

                .pwm_     = &htim2,
                .channel_ = TIM_CHANNEL_1,
                .us_tick_ = 0
        },
        [_PwmOutPin_2] = {

                .pwm_     = &htim2,
                .channel_ = TIM_CHANNEL_2,
                .us_tick_ = 0
        },
        [_PwmOutPin_3] = {

                .pwm_     = &htim2,
                .channel_ = TIM_CHANNEL_3,
                .us_tick_ = 0
        },
        [_PwmOutPin_4] = {

                .pwm_     = &htim2,
                .channel_ = TIM_CHANNEL_4,
                .us_tick_ = 0
        }
};

/* ****************************************************************************************************************** */

PwmOutHandle soc_pwmout_init(PwmOutPin pin, uint32_t freq)
{
    if (pin < _PwmOutPin_N)
    {
        uint32_t    clock_hz = HAL_RCC_GetPCLK1Freq() * 2;  // TIM2 is on the APB1 - 84MHz
        uint32_t    prescalar = 1;

        if (pwmout_data[pin].pwm_->State == HAL_TIM_STATE_RESET)
        {
            TIM_MasterConfigTypeDef     master_cfg;

            while (1) {

                if (((clock_hz / prescalar) / freq) <= PWM_MAX_PERIOD)
                    break;

                prescalar++;
            }

            clock_hz /= prescalar;

            pwmout_data[pin].pwm_->Init.Prescaler = prescalar - 1;
            pwmout_data[pin].pwm_->Init.Period    = (clock_hz / freq) - 1;

            HAL_TIM_OC_Init(pwmout_data[pin].pwm_);

            master_cfg.MasterOutputTrigger = TIM_TRGO_RESET;
            master_cfg.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

            HAL_TIMEx_MasterConfigSynchronization(pwmout_data[pin].pwm_, &master_cfg);
        }
        else
        {
            prescalar = pwmout_data[pin].pwm_->Init.Prescaler + 1;
            clock_hz /= prescalar;
        }

        TIM_OC_InitTypeDef  init_param;

        init_param.OCMode     = TIM_OCMODE_PWM1;
        init_param.Pulse      = 0;
        init_param.OCPolarity = TIM_OCPOLARITY_HIGH;
        init_param.OCFastMode = TIM_OCFAST_ENABLE;

        HAL_TIM_OC_ConfigChannel(pwmout_data[pin].pwm_, &init_param, pwmout_data[pin].channel_);
        HAL_TIM_OC_Start(pwmout_data[pin].pwm_, pwmout_data[pin].channel_);

        pwmout_data[pin].us_tick_ = (int16_t)(clock_hz / 1000000);

        return (PwmOutHandle)(&pwmout_data[pin]);
    }

    return NULL;
}

void soc_pwmout_set_duty(PwmOutHandle h, float duty_us)
{
    LocalDataOut    *this = (LocalDataOut*)h;

    __HAL_TIM_SET_COMPARE(this->pwm_, this->channel_, (uint32_t)(duty_us * (float)this->us_tick_));
}

/* end of file ****************************************************************************************************** */
