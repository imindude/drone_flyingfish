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

#include "pt1.h"
#include "maths.h"

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    float   a_;
    float   rc_;
    float   y_;
}
LocalData;

/* ****************************************************************************************************************** */

/**
 * PT1-Glied (RC) filter
 *
 * The highest cut-off freq that will have any affect is "fs / (2 * pi)"
 * cutoff_frequency = 1 / (2 * pi * RC)
 * RC = 1 / (2 * pi * cutoff_frequency)
 *
 * Return RC low-pass filter output samples, given input samples, time interval dt, and time constant RC function
 *
 * function lowpass(real[0..n] x, real dt, real RC)
 *     var real[0..n] y
 *     var real α := dt / (RC + dt)
 *     y[0] := x[0]
 *     for i from 1 to n
 *         y[i] := α * x[i] + (1-α) * y[i-1]
 *     return y
 *
 * The loop that calculates each of the n outputs can be refactored into the equivalent:
 *
 *     for i from 1 to n
 *         y[i] := y[i-1] + α * (x[i] - y[i-1])
 */

Pt1Handle pt1_init(int32_t fc_hz)
{
    LocalData   *this = (LocalData*)calloc(1, sizeof(LocalData));

    this->rc_ = 1.0f / (2.0f * M_PI * (float)fc_hz);

    return (Pt1Handle)this;
}

void pt1_reset(Pt1Handle h)
{
    LocalData   *this = (LocalData*)h;

    this->a_ = 0.0f;
    this->y_ = 0.0f;
}

float pt1_update(Pt1Handle h, float x, float dt)
{
    LocalData   *this = (LocalData*)h;

    this->a_  = dt / (this->rc_ + dt);
    this->y_ += this->a_ * (x - this->y_);

    return this->y_;
}

/* end of file ****************************************************************************************************** */
