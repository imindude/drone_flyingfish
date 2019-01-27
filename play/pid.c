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

#include "pid.h"
#include "pt1.h"
#include "maths.h"

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    PidGain     gain_;
    float       max_i_;
    float       angle_amp_;
    float       rate_amp_;
    Pt1Handle   dterm_lpf_;
    float       sum_i_;
    float       prev_rate_;
}
LocalData;

/* ****************************************************************************************************************** */

PidHandle pid_init(PidInitParam *init_param)
{
    LocalData   *this = (LocalData*)calloc(1, sizeof(LocalData));

    this->gain_      = init_param->gain_;
    this->max_i_     = (float)init_param->max_i_;
    this->angle_amp_ = (float)init_param->angle_amp_;
    /**
     * USER's rate range : 0 ~ 100
     * DPS range         : ABS // 90 ~ 1080
     * http://www.wolframalpha.com/input/?i=linear+fit+%7B0,+90%7D,+%7B100,+1080%7D
     */
    this->rate_amp_  = (float)init_param->rate_amp_ * 9.9f + 90.0f;

    if (init_param->dterm_fc_ > 0)
        this->dterm_lpf_ = pt1_init(init_param->dterm_fc_);

    return (PidHandle)this;
}

void pid_reset(PidHandle h)
{
    LocalData   *this = (LocalData*)h;

    this->sum_i_     = 0.0f;
    this->prev_rate_ = 0.0f;

    if (this->dterm_lpf_)
        pt1_reset(this->dterm_lpf_);
}

float pid_update(PidHandle h, ControlMode mode, float actual, float desire, float rate, float dt)
{
    LocalData   *this = (LocalData*)h;
    float   rate_err;
    float   angle_rate;
    float   p_term, i_term, d_term;
    float   delta;

    if (mode == _ControlMode_Angle)
        angle_rate = (desire - actual) * this->angle_amp_;
    else
        angle_rate = desire * this->rate_amp_;

    // P term

    rate_err = angle_rate - rate;
    p_term = rate_err * this->gain_.kp_;

    // I term

    this->sum_i_ += rate_err * dt * this->gain_.ki_;
    this->sum_i_ = CONSTRAIN(this->sum_i_, -this->max_i_, this->max_i_);
    i_term = this->sum_i_;

    // D term

    delta = (this->prev_rate_ - rate) / dt;
    if (this->dterm_lpf_)
        delta = pt1_update(this->dterm_lpf_, delta, dt);
    d_term = delta * this->gain_.kd_;

    this->prev_rate_ = rate;

    return p_term + i_term + d_term;
}

/* end of file ****************************************************************************************************** */
