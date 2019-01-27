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

#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

/* ****************************************************************************************************************** */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ****************************************************************************************************************** */

#define MAX_RC_CHANNELS         18
#define ANGLE_LIMIT_RPY_DEG     40

#define MAX_THR_VALUE           1.0f
#define MIN_THR_VALUE           0.0f
#define NEU_THR_VALUE           0.5f
#define MAX_RPY_VALUE           1.0f
#define MIN_RPY_VALUE           -1.0f
#define NEU_RPY_VALUE           0.0f

typedef enum _Axis
{
    _Axis_X,
    _Axis_Y,
    _Axis_Z,
    _Axis_N
}
Axis;

typedef enum _Euler
{
    _Euler_R,   // phi   : rotation around the X-axis
    _Euler_P,   // theta : rotation around the Y-axis
    _Euler_Y,   // psi   : rotation around the Z-axis
    _Euler_N
}
Euler;

typedef union _AxisI16
{
    int16_t     v_[_Axis_N];
    struct
    {
        int16_t x_, y_, z_;
    }
    xyz_;
}
AxisI16;

typedef union _AxisF32
{
    float       v_[_Axis_N];
    struct
    {
        float   x_, y_, z_;
    }
    xyz_;
}
AxisF32;

typedef union _EulerI16
{
    int16_t     v_[_Euler_N];
    struct
    {
        int16_t r_, p_, y_;
    }
    rpy_;
}
EulerI16;

typedef union _EulerF32
{
    float       v_[_Euler_N];
    struct
    {
        float   r_, p_, y_;
    }
    rpy_;
}
EulerF32;

typedef enum _MixerType
{
    _MixerType_X4,
    _MixerType_N
}
MixerType;

typedef enum _EscType
{
    _EscType_Brushed,       // 5KHz
    _EscType_Normal,        // 500Hz
    _EscType_OneShot125,    // 4KHz
    _EscType_OneShot42,     // 12KHz
    _EscType_MultiShot,     // 32KHz
    _EscType_N
}
EscType;

typedef enum _ReceiverType
{
    _ReceiverType_CPPM,
    _ReceiverType_SBUS,
    _ReceiverType_N
}
ReceiverType;

typedef enum _ControlMode
{
    _ControlMode_Angle,
    _ControlMode_Rate,
    _ControlMode_N
}
ControlMode;

typedef struct _PidGain
{
    float   kp_;
    float   ki_;
    float   kd_;
}
PidGain;

typedef struct _StickMap
{
    uint8_t thr_;
    uint8_t rol_;
    uint8_t pit_;
    uint8_t yaw_;
}
StickMap;

typedef struct _StickCmd
{
    float       thr_;
    float       rol_;
    float       pit_;
    float       yaw_;
    float       tol_;       // abs(tolerance)
    uint32_t    wait_ms_;
}
StickCmd;

typedef struct _SwitchCmd
{
    uint8_t aux_;
    float   min_;
    float   max_;
}
SwitchCmd;

typedef enum _Rotation
{
    _Rotation_CW_0,
    _Rotation_CW_90,
    _Rotation_CW_180,
    _Rotation_CW_270,
    _Rotation_N
}
Rotation;

typedef enum _LoopRate
{
    _LoopRate_Normal,
    _LoopRate_Fast,
    _LoopRate_N
}
LoopRate;

typedef void    *Handle;

/* ****************************************************************************************************************** */

extern uint32_t sys_get_millis(void);
extern uint32_t sys_get_micros(void);
extern void     sys_delay_millis(uint32_t ms);
extern void     sys_delay_micros(uint32_t us);

/* ****************************************************************************************************************** */

#endif /* __TYPEDEF_H__ */

/* end of file ****************************************************************************************************** */
