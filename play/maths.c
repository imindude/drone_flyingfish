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

#include "maths.h"

/* ****************************************************************************************************************** */

/**
 * The math here is done with help of: https://www.wolframalpha.com/input/?i=Solve%5By%3D((g%2F100)*x%5E3%2B((100-g)%2F100)*x),x%5D
 * e.g. y = 0.85 expo = 50% : https://www.wolframalpha.com/input/?i=Solve%5B0.85%3D((50%2F100)*x%5E3%2B((100-50)%2F100)*x),x%5D
 *
 *  x : input from [-1,1]
 *  g : sets the exponential amount [0,100]
 */
static float make_curve(float x, int8_t g)
{
    return ((float)g / 100.0f) * powf(x, 3) + ((float)(100 - g) / 100.0f) * x;
}

float rc_to_throttle(float rc, int8_t expo)
{
    rc = SCALE(rc, -1.0f, 1.0f, 0.0f, 1.0f);     // throttle range : 0 ~ 1
    rc = make_curve(rc, expo);

    return rc;
}

float rc_to_angle_deg(float rc, uint8_t max_deg, int8_t expo)
{
    rc = make_curve(rc, expo);
    rc = rc * (float)max_deg;

    return rc;
}

__attribute__((always_inline))
inline float inv_sqrtf(float x)
{
    float   halfx = 0.5f * x;
    float   y = x;
    long    i = *(int32_t*)&y;

    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

__attribute__((always_inline))
inline float rad_to_deg(float rad)
{
    float   deg = rad * (180.0f / M_PI);

    return deg;
}

__attribute__((always_inline))
inline float deg_to_rad(float deg)
{
    float   rad = deg * (M_PI / 180.0f);

    return rad;
}

EulerF32 get_euler_angle_from_quaternion(Quaternion q)
{
    EulerF32    euler;

    /**
     * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
     */

    // phi : rotation about the X-axis
    euler.rpy_.r_ = atan2f(2.0f * (q.q0_ * q.q1_ + q.q2_ * q.q3_), 1.0f - 2.0f * (q.q1_ * q.q1_ + q.q2_ * q.q2_));
    // theta : rotation about the Y-axis
    euler.rpy_.p_ = asinf(2.0f * (q.q0_ * q.q2_ - q.q1_ * q.q3_));
    // psi : rotation about the Z-axis
    euler.rpy_.y_ = atan2f(2.0f * (q.q0_ * q.q3_ + q.q1_ * q.q2_), 1.0f - 2.0f * (q.q2_ * q.q2_ + q.q3_ * q.q3_));

    return euler;
}

/* end of file ****************************************************************************************************** */
