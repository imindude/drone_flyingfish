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

#include "pilot.h"
#include "schedule.h"
#include "config.h"
#include "maths.h"
#include "ahrs.h"
#include "mixer.h"
#include "pid.h"
#include "receiver.h"

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    AhrsHandle      ahrs_;
    MixerHandle     mixer_;
    PidHandle       pid_[_Euler_N];
    ReceiverHandle  receiver_;

    uint32_t        last_exe_us_;
    bool            ahrs_update_;
    bool            receiver_update_;
    float           desired_thr_;
    EulerF32        desired_rpy_;

    StickCmd        sensor_cali_;
    uint32_t        sensor_cali_ms_;
}
LocalData;

/* ****************************************************************************************************************** */

static LocalData    pilot_data = {

        .last_exe_us_     = 0,
        .ahrs_update_     = false,
        .receiver_update_ = false,
        .desired_thr_     = 0.0f,
        .desired_rpy_.rpy_.r_ = 0.0f,
        .desired_rpy_.rpy_.p_ = 0.0f,
        .desired_rpy_.rpy_.y_ = 0.0f,

        .sensor_cali_.thr_     = MAX_THR_VALUE,
        .sensor_cali_.rol_     = MIN_RPY_VALUE,
        .sensor_cali_.pit_     = MAX_RPY_VALUE,
        .sensor_cali_.yaw_     = MAX_RPY_VALUE,
        .sensor_cali_.tol_     = 0.1f,
        .sensor_cali_.wait_ms_ = 1500,
        .sensor_cali_ms_       = 0
};

/* ****************************************************************************************************************** */

static void _receiver_callback(int8_t n_ch, bool failsafe, void *param)
{
    ((LocalData*)param)->receiver_update_ = true;
}

static void check_stick_command(LocalData *this, float thr, EulerF32 rpy, uint32_t now_ms)
{
    if (!get_ram()->armed_)
    {
        if (TOLERANCE(thr, this->sensor_cali_.thr_, this->sensor_cali_.tol_) &&
                TOLERANCE(rpy.rpy_.r_, this->sensor_cali_.rol_, this->sensor_cali_.tol_) &&
                TOLERANCE(rpy.rpy_.p_, this->sensor_cali_.pit_, this->sensor_cali_.tol_) &&
                TOLERANCE(rpy.rpy_.y_, this->sensor_cali_.yaw_, this->sensor_cali_.tol_))
        {
            // sensor calibration ready
            if (this->sensor_cali_ms_ == 0)
            {
                this->sensor_cali_ms_ = now_ms;
            }
            else if ((this->sensor_cali_ms_ + now_ms) >= this->sensor_cali_.wait_ms_)
            {
                this->sensor_cali_ms_ = 0;
                get_ram()->ahrs_ready_ = false;
                ahrs_sensor_calibration(this->ahrs_);
            }
        }
        else
        {
            this->sensor_cali_ms_ = 0;
        }
    }
}

static void check_switch_command(LocalData *this, float thr, EulerF32 rpy)
{
    float   val;

    val = receiver_get_key(this->receiver_, get_rom()->arming_cmd_.aux_);
    if (IN_RANGE(val, get_rom()->arming_cmd_.min_, get_rom()->arming_cmd_.max_))
    {
        if (get_ram()->armed_)
        {
            get_ram()->armed_ = false;
        }
        else if (get_ram()->ahrs_ready_ && (thr <= MIN_THR_VALUE))
        {
            get_ram()->armed_ = true;
        }
    }

    val = receiver_get_key(this->receiver_, get_rom()->rate_mode_cmd_.aux_);
    if (IN_RANGE(val, get_rom()->rate_mode_cmd_.min_, get_rom()->rate_mode_cmd_.max_))
    {
        get_ram()->ctrl_mode_ = _ControlMode_Rate;
    }
    else
    {
        get_ram()->ctrl_mode_ = _ControlMode_Angle;
    }
}

static void thread(uint32_t now_us, void *param)
{
    LocalData   *this = (LocalData*)param;
    float       dt = (now_us - this->last_exe_us_) / 1000000.0f;

    if (this->receiver_update_)
    {
        float       thr;
        EulerF32    rpy;

        thr = receiver_get_sticks(this->receiver_, &rpy);

        check_stick_command(this, thr, rpy, now_us);
        check_switch_command(this, thr, rpy);

        this->desired_thr_ = rc_to_throttle(this->desired_thr_, get_rom()->thr_expo_);
        for (int8_t i = 0; i < _Euler_N; i++)
            this->desired_rpy_.v_[i] =
                    rc_to_angle_deg(this->desired_rpy_.v_[i], ANGLE_LIMIT_RPY_DEG, get_rom()->rpy_expo_[i]);

        this->receiver_update_ = false;
    }

    if (this->ahrs_update_)
    {
        EulerF32    actuator_rpy = {

                .rpy_.r_ = 0.0f,
                .rpy_.p_ = 0.0f,
                .rpy_.y_ = 0.0f
        };
        AxisF32     gyro_dps, acc_g;
        EulerF32    actual_rpy = ahrs_get_motion_update(this->ahrs_, &gyro_dps, &acc_g, dt);

        if (get_ram()->ahrs_ready_ == false)
        {
            if ((actual_rpy.rpy_.r_ != 0.0f) || (actual_rpy.rpy_.p_ != 0.0f) || (actual_rpy.rpy_.y_ != 0.0f))
                get_ram()->ahrs_ready_ = true;
        }

        if (get_ram()->armed_ && get_ram()->ahrs_ready_)
        {
            actuator_rpy.v_[_Euler_R] = pid_update(this->pid_[_Euler_R], get_ram()->ctrl_mode_, actual_rpy.v_[_Euler_R],
                    this->desired_rpy_.v_[_Euler_R], gyro_dps.v_[_Euler_R], dt);
        }
        else
        {
            for (int8_t i = 0; i < _Euler_N; i++)
                pid_reset(this->pid_[i]);
        }

        mixer_output(this->mixer_, this->desired_thr_, actuator_rpy);

        this->ahrs_update_ = false;
    }

    this->last_exe_us_ = now_us;
}

static bool wakeup(uint32_t now_us, void *param)
{
    LocalData   *this = (LocalData*)param;

    this->ahrs_update_ = ahrs_check_motion_update(this->ahrs_, now_us);

    if (this->ahrs_update_ || this->receiver_update_)
        return true;

    return false;
}

void pilot_init(void)
{
    RomData         *rom = get_rom();

    config_init();

    pilot_data.ahrs_ =
                ahrs_init(rom->loop_rate_, rom->gyro_rot_, rom->acc_rot_, rom->acc_bias_, rom->gyro_fc_, rom->acc_fc_);
    pilot_data.mixer_ = mixer_init(rom->mixer_type_, rom->esc_type_);
    pilot_data.receiver_ = receiver_init(rom->receiver_type_, rom->stick_map_, _receiver_callback, &pilot_data);

    for (int8_t i = 0; i < _Euler_N; i++)
    {
        PidInitParam    pid_param = {

                .gain_      = get_rom()->pid_gain_[i],
                .max_i_     = get_rom()->max_iterm_[i],
                .angle_amp_ = get_rom()->angle_amp_[i],
                .rate_amp_  = get_rom()->rate_amp_[i],
                .dterm_fc_  = get_rom()->dterm_fc_[i]
        };

        pilot_data.pid_[i] = pid_init(&pid_param);
    }

    schedule_signup(thread, wakeup, &pilot_data, _SchedulePrio_UserHigh);
}

/* end of file ****************************************************************************************************** */
