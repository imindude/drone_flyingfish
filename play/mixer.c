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

#include "mixer.h"
#include "maths.h"
#include "drv_motor.h"

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    MotorHandle motors_[_MotorDev_N];
    int8_t      n_motor_;
    EulerF32    mixing_rate_[_MotorDev_N];
}
LocalData;

/* ****************************************************************************************************************** */

MixerHandle mixer_init(MixerType mixer, EscType esc)
{
    if (mixer == _MixerType_X4) {

        LocalData   *this;
        MotorDuty   duty;

        switch (esc)
        {
        case _EscType_Brushed:
            duty = _MotorDuty_0_200_5K;
            break;
        case _EscType_Normal:
            duty = _MotorDuty_1K_2K_500;
            break;
        case _EscType_OneShot125:
            duty = _MotorDuty_125_250_4K;
            break;
        case _EscType_OneShot42:
            duty = _MotorDuty_42_84_8K;
            break;
        case _EscType_MultiShot:
            duty = _MotorDuty_5_25_32K;
            break;
        default:
            return NULL;
        }

        this = (LocalData*)calloc(1, sizeof(LocalData));
        this->n_motor_ = 4;     // quad

        // Motor 1 : Right - Rear - CW

        this->motors_[_MotorDev_1] = drv_motor_init(_MotorDev_1, duty);
        drv_motor_out(this->motors_[_MotorDev_1], 0.0f);

        this->mixing_rate_[_MotorDev_1].rpy_.r_ = -1.0f;
        this->mixing_rate_[_MotorDev_1].rpy_.p_ = -1.0f;
        this->mixing_rate_[_MotorDev_1].rpy_.y_ =  1.0f;

        // Motor 2 : Right - Front - CCW

        this->motors_[_MotorDev_2] = drv_motor_init(_MotorDev_2, duty);
        drv_motor_out(this->motors_[_MotorDev_2], 0.0f);

        this->mixing_rate_[_MotorDev_2].rpy_.r_ = -1.0f;
        this->mixing_rate_[_MotorDev_2].rpy_.p_ =  1.0f;
        this->mixing_rate_[_MotorDev_2].rpy_.y_ = -1.0f;

        // Motor 3 : Left - Rear - CCW

        this->motors_[_MotorDev_3] = drv_motor_init(_MotorDev_3, duty);
        drv_motor_out(this->motors_[_MotorDev_3], 0.0f);

        this->mixing_rate_[_MotorDev_3].rpy_.r_ =  1.0f;
        this->mixing_rate_[_MotorDev_3].rpy_.p_ = -1.0f;
        this->mixing_rate_[_MotorDev_3].rpy_.y_ = -1.0f;

        // Motor 4 : Left - Rear - CW

        this->motors_[_MotorDev_4] = drv_motor_init(_MotorDev_4, duty);
        drv_motor_out(this->motors_[_MotorDev_4], 0.0f);

        this->mixing_rate_[_MotorDev_4].rpy_.r_ =  1.0f;
        this->mixing_rate_[_MotorDev_4].rpy_.p_ =  1.0f;
        this->mixing_rate_[_MotorDev_4].rpy_.y_ =  1.0f;

        return (MixerHandle)this;
    }

    return NULL;
}

void mixer_output(MixerHandle h, float throttle, EulerF32 actuator)
{
    LocalData   *this = (LocalData*)h;
    float   actual_thrust[this->n_motor_];
    float   max_thrust = 0.0f;
    float   overshoot = 0.0f;
    int8_t  i;

    for (i = 0; i < this->n_motor_; i++)
    {
        actual_thrust[i] = throttle;
        for (int8_t n = 0; n < _Euler_N; n++)
            actual_thrust[i] += actuator.v_[n] * this->mixing_rate_[i].v_[n];

        if (max_thrust < actual_thrust[i])
            max_thrust = actual_thrust[i];
    }

    if (max_thrust > 1.0f)
        overshoot = max_thrust - 1.0f;

    for (i = 0; i < this->n_motor_; i++)
    {
        actual_thrust[i] -= overshoot;
        actual_thrust[i] = CONSTRAIN(actual_thrust[i], 0.0f, 1.0f);
        drv_motor_out(this->motors_[i], actual_thrust[i]);
    }
}

/* end of file ****************************************************************************************************** */
