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

#include "wflc.h"

/* ****************************************************************************************************************** */

#define WFLC_N_MARMONICS        2

enum
{
    _Vector_Sin,
    _Vector_Cos,
    _Vector_N
};

typedef struct _LocalData
{
    float   mu0_;       // μ0: adaptive gain for fundamental frequency
    float   mu1_;       // μ1: adaptive gain for reference input vector
    float   omega0_;    // ω0: fundamental angular frequency
    float   wv_[_Vector_N][WFLC_N_MARMONICS];   // weight vector, 1st row for sin and 2nd row for cos
}
LocalData;

/* ****************************************************************************************************************** */

/**
 * Weighted-frequency Fourier Linear Combiner
 * https://github.com/yan9a/Adaptive_Filters/tree/master/Filter_WFLC
 */

WflcHandle wflc_init(float mu0, float mu1, float omega0)
{
    LocalData   *this = (LocalData*)calloc(1, sizeof(LocalData));

    this->mu0_    = mu0;
    this->mu1_    = mu1;
    this->omega0_ = omega0;

    return (WflcHandle)this;
}

void wflc_reset(WflcHandle h, float omega0)
{
    LocalData   *this = (LocalData*)h;

    this->omega0_ = omega0;

    for (int8_t i = 0; i < WFLC_N_MARMONICS; i++) {

        this->wv_[_Vector_Sin][i] = 0.0f;
        this->wv_[_Vector_Cos][i] = 0.0f;
    }
}

float wflc_update(WflcHandle h, float x, float dt)
{
    LocalData   *this = (LocalData*)h;
    int8_t  i;
    float   err, y = 0.0f, z = 0.0f;
    float   af[WFLC_N_MARMONICS];               // array of angular frequencies
    float   iv[_Vector_N][WFLC_N_MARMONICS];    // reference input vector, 1st row for sin and 2nd row for cos

    // Get angular velocities depending on adjusted fundamental angular frequency
    for (i = 0; i < WFLC_N_MARMONICS; i++)
        af[i] = (i + 1) * this->omega0_;        // assign ω0 and its harmonics

    // find reference input vector
    for (i = 0; i < WFLC_N_MARMONICS; i++) {

        iv[_Vector_Sin][i] = sinf(af[i] * dt);
        iv[_Vector_Cos][i] = sinf(af[i] * dt);
    }

    // find estimated signal, y
    for (i = 0; i < WFLC_N_MARMONICS; i++)
        y += this->wv_[_Vector_Sin][i] * iv[_Vector_Sin][i] + this->wv_[_Vector_Cos][i] * iv[_Vector_Cos][i];

    // adapt the weights
    err = x - y;
    for (i = 0; i < WFLC_N_MARMONICS; i++)
        z += (i + 1) * (this->wv_[_Vector_Sin][i] * iv[_Vector_Cos][i] - this->wv_[_Vector_Cos][i] * iv[_Vector_Sin][i]);

    this->omega0_ = this->omega0_ + 2.0f * this->mu0_ * err * z;

    for (i = 0; i < WFLC_N_MARMONICS; i++) {

        this->wv_[_Vector_Sin][i] += 2.0f * this->mu1_ * iv[_Vector_Sin][i] * err;
        this->wv_[_Vector_Cos][i] += 2.0f * this->mu1_ * iv[_Vector_Cos][i] * err;
    }

    return y;
}

/* end of file ****************************************************************************************************** */
