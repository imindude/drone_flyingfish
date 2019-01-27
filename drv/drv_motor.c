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

/* ****************************************************************************************************************** */

#include "drv_motor.h"
#include "soc_pwm.h"

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    PwmOutHandle    pwm_;
    float   scale_mul_;
    float   scale_add_;
}
LocalData;

/* ****************************************************************************************************************** */

static LocalData    motor_data[_MotorDev_N] = {

        [_MotorDev_1] = {

                .pwm_       = NULL,
                .scale_mul_ = 0,
                .scale_add_ = 0
        },
        [_MotorDev_2] = {

                .pwm_       = NULL,
                .scale_mul_ = 0,
                .scale_add_ = 0
        },
        [_MotorDev_3] = {

                .pwm_       = NULL,
                .scale_mul_ = 0,
                .scale_add_ = 0
        },
        [_MotorDev_4] = {

                .pwm_       = NULL,
                .scale_mul_ = 0,
                .scale_add_ = 0
        }
};

/* ****************************************************************************************************************** */

MotorHandle drv_motor_init(MotorDev dev, MotorDuty duty)
{
    if (dev < _MotorDev_N)
    {
        PwmOutPin   pwm_pin_list[_MotorDev_N] = {

                [_MotorDev_1] = _PwmOutPin_1,
                [_MotorDev_2] = _PwmOutPin_2,
                [_MotorDev_3] = _PwmOutPin_3,
                [_MotorDev_4] = _PwmOutPin_4
        };
        uint32_t    freq;

        switch (duty)
        {
        case _MotorDuty_0_200_5K:       // brushed motor
            freq = 5000;                // 0 ~ 200us
            // https://www.wolframalpha.com/input/?i=linear+fit+%7B0,+0%7D,+%7B1,+200%7D
            motor_data[dev].scale_mul_ = 200.0f;
            motor_data[dev].scale_add_ = 0.0f;
            break;
        case _MotorDuty_1K_2K_500:      // normal pwm
            freq = 500;                 // 1000 ~ 2000[1900]us
            // https://www.wolframalpha.com/input/?i=linear+fit+%7B0,+1000%7D,+%7B1,+1900%7D
            motor_data[dev].scale_mul_ = 900.0f;
            motor_data[dev].scale_add_ = 1000.0f;
            break;
        case _MotorDuty_125_250_4K:     // oneshot125
            freq = 4000;                // 125 ~ 250[245]us
            // https://www.wolframalpha.com/input/?i=linear+fit+%7B0,+125%7D,+%7B1,+245%7D
            motor_data[dev].scale_mul_ = 120.0f;
            motor_data[dev].scale_add_ = 125.0f;
            break;
        case _MotorDuty_42_84_8K:       // oneshot42 (max 11.9K)
            freq = 8000;                // 42 ~ 84us
            // https://www.wolframalpha.com/input/?i=linear+fit+%7B0,+42%7D,+%7B1,+84%7D
            motor_data[dev].scale_mul_ = 42.0f;
            motor_data[dev].scale_add_ = 42.0f;
            break;
        case _MotorDuty_5_25_32K:       // multishot (max 40K)
            freq = 32000;               // 5 ~ 25us
            // https://www.wolframalpha.com/input/?i=linear+fit+%7B0,+5%7D,+%7B1,+25%7D
            motor_data[dev].scale_mul_ = 20.0f;
            motor_data[dev].scale_add_ = 5.0f;
            break;
        default:
            return NULL;
        }

        motor_data[dev].pwm_ = soc_pwmout_init(pwm_pin_list[dev], freq);

        return (MotorHandle)(&motor_data[dev]);
    }

    return NULL;
}

void drv_motor_out(MotorHandle h, float duty)
{
    LocalData   *this = (LocalData*)h;

    soc_pwmout_set_duty(this->pwm_, duty * this->scale_mul_ + this->scale_add_);
}

/* end of file ****************************************************************************************************** */
