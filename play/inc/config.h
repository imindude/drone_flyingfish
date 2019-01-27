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

#ifndef __CONFIG_H__
#define __CONFIG_H__

/* ****************************************************************************************************************** */

#include "typedef.h"

/* ****************************************************************************************************************** */

#pragma pack(push, 1)
typedef struct _RomData
{
    LoopRate        loop_rate_;
    Rotation        gyro_rot_;
    Rotation        acc_rot_;
    AxisI16         acc_bias_;
    AxisI16         gyro_fc_;
    AxisI16         acc_fc_;

    EscType         esc_type_;
    MixerType       mixer_type_;

    PidGain         pid_gain_[_Euler_N];
    int8_t          max_iterm_[_Euler_N];
    int8_t          angle_amp_[_Euler_N];
    int8_t          rate_amp_[_Euler_N];
    int16_t         dterm_fc_[_Euler_N];

    ReceiverType    receiver_type_;
    StickMap        stick_map_;
    int8_t          thr_expo_;
    int8_t          rpy_expo_[_Euler_N];

    SwitchCmd       arming_cmd_;
    SwitchCmd       rate_mode_cmd_;
}
RomData;
#pragma pack(pop)

typedef struct _RamData
{
    bool        armed_;
    bool        ahrs_ready_;
    ControlMode ctrl_mode_;
}
RamData;

/* ****************************************************************************************************************** */

extern void     config_init(void);
extern RomData* get_rom(void);
extern RamData* get_ram(void);

/* ****************************************************************************************************************** */

#endif /* __CONFIG_H__ */

/* end of file ****************************************************************************************************** */
