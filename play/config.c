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

#include "config.h"

/* ****************************************************************************************************************** */

static RomData  ROM;
static RamData  RAM;

/* ****************************************************************************************************************** */

void config_init(void)
{
    ROM.loop_rate_ = _LoopRate_Normal;
    ROM.gyro_rot_  = _Rotation_CW_0;
    ROM.acc_rot_   = _Rotation_CW_0;

    ROM.acc_bias_.xyz_.x_ = 0;
    ROM.acc_bias_.xyz_.y_ = 0;
    ROM.acc_bias_.xyz_.z_ = 0;

    ROM.gyro_fc_.xyz_.x_ = 100;
    ROM.gyro_fc_.xyz_.y_ = 100;
    ROM.gyro_fc_.xyz_.z_ = 100;

    ROM.acc_fc_.xyz_.x_ = 100;
    ROM.acc_fc_.xyz_.y_ = 100;
    ROM.acc_fc_.xyz_.z_ = 100;

    ROM.esc_type_   = _EscType_Normal;
    ROM.mixer_type_ = _MixerType_X4;

    ROM.pid_gain_[_Euler_R].kp_ = 10.0f;
    ROM.pid_gain_[_Euler_R].ki_ = 10.0f;
    ROM.pid_gain_[_Euler_R].kd_ = 10.0f;

    ROM.pid_gain_[_Euler_P].kp_ = 10.0f;
    ROM.pid_gain_[_Euler_P].ki_ = 10.0f;
    ROM.pid_gain_[_Euler_P].kd_ = 10.0f;

    ROM.pid_gain_[_Euler_Y].kp_ = 10.0f;
    ROM.pid_gain_[_Euler_Y].ki_ = 10.0f;
    ROM.pid_gain_[_Euler_Y].kd_ = 0.0f;

    ROM.max_iterm_[_Euler_R] = 25;
    ROM.max_iterm_[_Euler_P] = 25;
    ROM.max_iterm_[_Euler_Y] = 25;

    ROM.angle_amp_[_Euler_R] = 50;
    ROM.angle_amp_[_Euler_P] = 50;
    ROM.angle_amp_[_Euler_Y] = 50;

    ROM.rate_amp_[_Euler_R] = 50;
    ROM.rate_amp_[_Euler_P] = 50;
    ROM.rate_amp_[_Euler_Y] = 50;

    ROM.dterm_fc_[_Euler_R] = 30;
    ROM.dterm_fc_[_Euler_P] = 30;
    ROM.dterm_fc_[_Euler_Y] = 30;

    ROM.receiver_type_ = _ReceiverType_SBUS;

    ROM.stick_map_.thr_ = 0;
    ROM.stick_map_.rol_ = 1;
    ROM.stick_map_.pit_ = 2;
    ROM.stick_map_.yaw_ = 3;

    ROM.thr_expo_ = 70;
    ROM.rpy_expo_[_Euler_R] = 70;
    ROM.rpy_expo_[_Euler_P] = 70;
    ROM.rpy_expo_[_Euler_Y] = 70;

    ROM.arming_cmd_.aux_ = 5;
    ROM.arming_cmd_.min_ = 0.7f;
    ROM.arming_cmd_.max_ = 1.0f;

    ROM.rate_mode_cmd_.aux_ = 6;
    ROM.rate_mode_cmd_.min_ = 0.7f;
    ROM.rate_mode_cmd_.max_ = 1.0f;

    // RAM

    RAM.armed_      = false;
    RAM.ahrs_ready_ = false;
    RAM.ctrl_mode_  = _ControlMode_Angle;
}

inline RomData* get_rom(void)
{
    return &ROM;
}

inline RamData* get_ram(void)
{
    return &RAM;
}

/* end of file ****************************************************************************************************** */
