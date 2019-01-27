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

#ifndef __MATHS_H__
#define __MATHS_H__

/* ****************************************************************************************************************** */

#include <math.h>
#include "typedef.h"

/* ****************************************************************************************************************** */

typedef struct _Quaternion
{
    float   q0_, q1_, q2_, q3_;
}
Quaternion;

/* ****************************************************************************************************************** */

#define SCALE(in, in_min, in_max, out_min, out_max) \
        (((((out_max) - (out_min)) * ((in) - (in_min))) / ((in_max) - (in_min))) + (out_min))
#define CONSTRAIN(in, min, max)     (((in) < (min)) ? (min) : ((in) > (max)) ? (max) : (in))
#define IN_RANGE(in, min, max)      (((in) >= (min)) && ((in) <= (max)))
#define TOLERANCE(in, val, tol)     (((in) >= ((val) - (tol))) && ((in) <= ((val) + (tol))))

#define square(x)                   ((x) * (x))

#define quaternion_reset(q)         {   \
        (q).q0_ = 1.0f;                 \
        (q).q1_ = 0.0f;                 \
        (q).q2_ = 0.0f;                 \
        (q).q3_ = 0.0f;                 \
}

extern float    inv_sqrtf(float x);
extern float    rc_to_throttle(float rc, int8_t expo);
extern float    rc_to_angle_deg(float rc, uint8_t max_deg, int8_t expo);
extern float    rad_to_deg(float rad);
extern float    deg_to_rad(float deg);
extern EulerF32 get_euler_angle_from_quaternion(Quaternion q);

/* ****************************************************************************************************************** */

#endif /* __MATHS_H__ */

/* end of file ****************************************************************************************************** */
