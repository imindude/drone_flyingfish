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

#include "ahrs.h"
#include "drv_mpu9255.h"
#include "pt1.h"
#include "maths.h"

/* ****************************************************************************************************************** */

#define MAHONY_TWO_KP       0.25f
#define MAHONY_TWO_KI       0.0f

typedef enum _CaliType
{
    _CaliType_None,
    _CaliType_Gyro,
    _CaliType_Acc,
    _CaliType_N
}
CaliType;

typedef struct _LocalData
{
    MpuHandle   mpu_;
    Rotation    gyro_rot_;
    Rotation    acc_rot_;
    AxisI16     gyro_bias_;
    AxisI16     acc_bias_;
    Pt1Handle   gyro_lpf_[_Axis_N];
    Pt1Handle   acc_lpf_[_Axis_N];

    float       gyro_factor_;
    float       acc_factor_;

    CaliType    cali_type_;
    int16_t     bias_try_max_;
    int16_t     bias_cnt_;
    AxisF32     bias_gyro_mean_;
    AxisF32     bias_acc_mean_;
    AxisI16     bias_gyro_min_;
    AxisI16     bias_gyro_max_;

    float       two_kp_;
    float       two_ki_;
    float       integral_feedback_x_;
    float       integral_feedback_y_;
    float       integral_feedback_z_;
    Quaternion  q_;
}
LocalData;

/* ****************************************************************************************************************** */

static bool bias_update(LocalData *this, AxisI16 sens_gyro, AxisI16 sens_acc)
{
    float   gain;

    this->bias_cnt_++;
    gain = 1.0f / (float)this->bias_cnt_;

    for (int8_t i = 0; i < _Axis_N; i++)
    {
        if (sens_gyro.v_[i] < this->bias_gyro_min_.v_[i])
            this->bias_gyro_min_.v_[i] = sens_gyro.v_[i];
        if (sens_gyro.v_[i] > this->bias_gyro_max_.v_[i])
            this->bias_gyro_max_.v_[i] = sens_gyro.v_[i];

        this->bias_gyro_mean_.v_[i] = this->bias_gyro_mean_.v_[i] * (1.0f - gain) + (float)sens_gyro.v_[i] * gain;
        this->bias_acc_mean_.v_[i]  = this->bias_acc_mean_.v_[i] * (1.0f - gain) + (float)sens_acc.v_[i] * gain;
    }

    if (this->bias_cnt_ >= this->bias_try_max_)
    {
        float       zero_tolerance = drv_mpu_get_gyro_zero_tolerance(this->mpu_);
        AxisF32     bias_tolerance;

        for (int8_t i = 0; i < _Axis_N; i++)
            bias_tolerance.v_[i] = (float)(this->bias_gyro_max_.v_[i] - this->bias_gyro_min_.v_[i]);

        if ((bias_tolerance.xyz_.x_ < zero_tolerance) &&
            (bias_tolerance.xyz_.y_ < zero_tolerance) &&
            (bias_tolerance.xyz_.z_ < zero_tolerance)) {

            return true;
        }
        else {

            this->bias_cnt_ = 0;

            for (int8_t i = 0; i < _Axis_N; i++)
            {
                this->bias_gyro_mean_.v_[i] = 0.0f;
                this->bias_acc_mean_.v_[i]  = 0.0f;
                this->bias_gyro_min_.v_[i]  = 0.0f;
                this->bias_gyro_max_.v_[i]  = 0.0f;
            }
        }
    }

    return false;
}

static void sensor_calibration_update(LocalData *this, AxisI16 sens_gyro, AxisI16 sens_acc)
{
    if (this->cali_type_ == _CaliType_Gyro)
    {
        if (bias_update(this, sens_gyro, sens_acc))
        {
            for (int8_t i = 0; i < _Axis_N; i++)
                this->gyro_bias_.v_[i] = (int16_t)this->bias_gyro_mean_.v_[i];

            this->integral_feedback_x_ = 0.0f;
            this->integral_feedback_y_ = 0.0f;
            this->integral_feedback_z_ = 0.0f;
            quaternion_reset(this->q_);

            this->cali_type_ = _CaliType_None;
        }
    }
    else if (this->cali_type_ == _CaliType_Acc)
    {
        if (bias_update(this, sens_gyro, sens_acc))
        {
            for (int8_t i = 0; i < _Axis_N; i++)
            {
                this->gyro_bias_.v_[i] = (int16_t)this->bias_gyro_mean_.v_[i];
                this->acc_bias_.v_[i]  = (int16_t)this->bias_acc_mean_.v_[i];
            }

            // remove gravity - it need 6-axis calibration to increase the accuracy
            this->acc_bias_.xyz_.z_ -= (int16_t)(1.0f / this->acc_factor_);

            this->integral_feedback_x_ = 0.0f;
            this->integral_feedback_y_ = 0.0f;
            this->integral_feedback_z_ = 0.0f;
            quaternion_reset(this->q_);

            this->cali_type_ = _CaliType_None;
        }
    }
}

static void apply_rotated_value(Rotation rotation, AxisI16 *sens_value)
{
    int16_t x = sens_value->xyz_.x_;
    int16_t y = sens_value->xyz_.y_;

    switch (rotation)
    {
    default:
    case _Rotation_CW_0:
        sens_value->xyz_.x_ = x;
        sens_value->xyz_.y_ = y;
        break;
    case _Rotation_CW_90:
        sens_value->xyz_.x_ = -y;
        sens_value->xyz_.y_ =  x;
        break;
    case _Rotation_CW_180:
        sens_value->xyz_.x_ = -x;
        sens_value->xyz_.y_ = -y;
        break;
    case _Rotation_CW_270:
        sens_value->xyz_.x_ =  y;
        sens_value->xyz_.y_ = -x;
        break;
    }
}

/**
 * Implementation of Madgwick's IMU and AHRS algorithms.
 * "http://www.x-io.co.uk/open-source-ahrs-with-x-imu/"
 */
static void ahrs_update(LocalData *this, AxisF32 gyro, AxisF32 acc, float dt)
{
    float   normalise;
    float   halfvx, halfvy, halfvz;
    float   halfex, halfey, halfez;
    float   qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((acc.xyz_.x_ == 0.0f) && (acc.xyz_.y_ == 0.0f) && (acc.xyz_.z_ == 0.0f)))
    {
        // Normalise accelerometer measurement
        normalise = inv_sqrtf(square(acc.xyz_.x_) + square(acc.xyz_.y_) + square(acc.xyz_.z_));
        acc.xyz_.x_ *= normalise;
        acc.xyz_.y_ *= normalise;
        acc.xyz_.z_ *= normalise;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = this->q_.q1_ * this->q_.q3_ - this->q_.q0_ * this->q_.q2_;
        halfvy = this->q_.q0_ * this->q_.q1_ + this->q_.q2_ * this->q_.q3_;
        halfvz = this->q_.q0_ * this->q_.q0_ - 0.5f + this->q_.q3_ * this->q_.q3_;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (acc.xyz_.y_ * halfvz - acc.xyz_.z_ * halfvy);
        halfey = (acc.xyz_.z_ * halfvx - acc.xyz_.x_ * halfvz);
        halfez = (acc.xyz_.x_ * halfvy - acc.xyz_.y_ * halfvx);

        // Compute and apply integral feedback if enabled
        if (this->two_ki_ > 0.0f) {

            // integral error scaled by Ki
            this->integral_feedback_x_ += this->two_ki_ * halfex * dt;
            this->integral_feedback_y_ += this->two_ki_ * halfey * dt;
            this->integral_feedback_z_ += this->two_ki_ * halfez * dt;

            // apply integral feedback
            gyro.xyz_.x_ += this->integral_feedback_x_;
            gyro.xyz_.y_ += this->integral_feedback_y_;
            gyro.xyz_.z_ += this->integral_feedback_z_;
        }
        else {

            // prevent integral windup
            this->integral_feedback_x_ = 0.0f;
            this->integral_feedback_y_ = 0.0f;
            this->integral_feedback_z_ = 0.0f;
        }

        // Apply proportional feedback
        gyro.xyz_.x_ += this->two_kp_ * halfex;
        gyro.xyz_.y_ += this->two_kp_ * halfey;
        gyro.xyz_.z_ += this->two_kp_ * halfez;
    }

    // Integrate rate of change of quaternion
    gyro.xyz_.x_ *= (0.5f * dt);
    gyro.xyz_.y_ *= (0.5f * dt);
    gyro.xyz_.z_ *= (0.5f * dt);
    qa = this->q_.q0_;
    qb = this->q_.q1_;
    qc = this->q_.q2_;
    this->q_.q0_ += (-qb * gyro.xyz_.x_ - qc * gyro.xyz_.y_ - this->q_.q3_ * gyro.xyz_.z_);
    this->q_.q1_ += ( qa * gyro.xyz_.x_ + qc * gyro.xyz_.z_ - this->q_.q3_ * gyro.xyz_.y_);
    this->q_.q2_ += ( qa * gyro.xyz_.y_ - qb * gyro.xyz_.z_ + this->q_.q3_ * gyro.xyz_.x_);
    this->q_.q3_ += ( qa * gyro.xyz_.z_ + qb * gyro.xyz_.y_ - qc * gyro.xyz_.x_);

    // Normalise quaternion
    normalise = inv_sqrtf(square(this->q_.q0_) + square(this->q_.q1_) + square(this->q_.q2_) + square(this->q_.q3_));
    this->q_.q0_ *= normalise;
    this->q_.q1_ *= normalise;
    this->q_.q2_ *= normalise;
    this->q_.q3_ *= normalise;
}

AhrsHandle ahrs_init(LoopRate rate, Rotation gyro_rot, Rotation acc_rot, AxisI16 acc_bias, AxisI16 gyro_fc, AxisI16 acc_fc)
{
    LocalData   *this = (LocalData*)calloc(1, sizeof(LocalData));

    this->mpu_ = drv_mpu_init(_MpuDev_1, (rate == _LoopRate_Normal) ? _MpuMode_1K : _MpuMode_8K);
    this->gyro_rot_ = gyro_rot;
    this->acc_rot_  = acc_rot;
    this->acc_bias_ = acc_bias;

    for (int8_t i = 0; i < _Axis_N; i++)
    {
        this->gyro_lpf_[i] = pt1_init(gyro_fc.v_[i]);
        this->acc_lpf_[i]  = pt1_init(acc_fc.v_[i]);
    }

    this->gyro_factor_ = drv_mpu_get_gyro_factor(this->mpu_);
    this->acc_factor_  = drv_mpu_get_acc_factor(this->mpu_);

//    this->cali_type_    = _CaliType_Gyro;
    this->cali_type_    = _CaliType_Acc;
    this->bias_try_max_ = (rate == _LoopRate_Normal) ? 1500 : 12000;

    this->two_kp_ = MAHONY_TWO_KP;
    this->two_ki_ = MAHONY_TWO_KI;

    quaternion_reset(this->q_);

    return (AhrsHandle)this;
}

void ahrs_sensor_calibration(AhrsHandle h)
{
    LocalData   *this = (LocalData*)h;

    this->cali_type_ = _CaliType_Acc;
    this->bias_cnt_  = 0;

    for (int8_t i = 0; i < _Axis_N; i++)
    {
        this->bias_gyro_mean_.v_[i] = 0.0f;
        this->bias_acc_mean_.v_[i]  = 0.0f;
        this->bias_gyro_min_.v_[i]  = 0.0f;
        this->bias_gyro_max_.v_[i]  = 0.0f;
    }
}

bool ahrs_check_motion_update(AhrsHandle h, uint32_t now_us)
{
    LocalData   *this = (LocalData*)h;
    bool        ready = drv_mpu_is_ready(this->mpu_, now_us);

    return ready;
}

int debug_count = 0;
EulerF32 ahrs_get_motion_update(AhrsHandle h, AxisF32 *gyro, AxisF32 *acc, float dt)
{
    LocalData   *this = (LocalData*)h;
    AxisI16     sens_gyro, sens_acc;
    EulerF32    euler_angle = {

            .rpy_.r_ = 0.0f,
            .rpy_.p_ = 0.0f,
            .rpy_.y_ = 0.0f
    };

    drv_mpu_read_data(this->mpu_, &sens_gyro.v_[_Axis_X], &sens_gyro.v_[_Axis_Y], &sens_gyro.v_[_Axis_Z],
            &sens_acc.v_[_Axis_X], &sens_acc.v_[_Axis_Y], &sens_acc.v_[_Axis_Z]);

    if (this->cali_type_ == _CaliType_None)
    {
        int8_t      i;
        AxisF32     gyro_dps, acc_g;
        AxisF32     gyro_rad;

        for (i = 0; i < _Axis_N; i++)
        {
            sens_gyro.v_[i] -= this->gyro_bias_.v_[i];
            sens_acc.v_[i]  -= this->acc_bias_.v_[i];
        }

        apply_rotated_value(this->gyro_rot_, &sens_gyro);
        apply_rotated_value(this->acc_rot_, &sens_acc);

        for (i = 0; i < _Axis_N; i++)
        {
            gyro_dps.v_[i] = (float)sens_gyro.v_[i] * this->gyro_factor_;
            gyro_dps.v_[i] = pt1_update(this->gyro_lpf_[i], gyro_dps.v_[i], dt);

            gyro_rad.v_[i] = deg_to_rad(gyro_dps.v_[i]);

            acc_g.v_[i] = (float)sens_acc.v_[i] * this->acc_factor_;
            acc_g.v_[i] = pt1_update(this->acc_lpf_[i], acc_g.v_[i], dt);
        }

        ahrs_update(this, gyro_rad, acc_g, dt);
        euler_angle = get_euler_angle_from_quaternion(this->q_);

        for (i = 0; i < _Axis_N; i++)
            euler_angle.v_[i] = rad_to_deg(euler_angle.v_[i]);

        *gyro = gyro_dps;
        *acc  = acc_g;
    }
    else
    {
        sensor_calibration_update(this, sens_gyro, sens_acc);
    }

    if (debug_count++ > 40)
    {
        extern int16_t VCP_Tx(uint8_t *buffer, int16_t size, uint32_t timeout_ms);
        debug_count = 0;

#if 1
        int8_t  buf[3] = { (int8_t)euler_angle.v_[0], (int8_t)euler_angle.v_[1], (int8_t)euler_angle.v_[2] };
        VCP_Tx((uint8_t*)buf, 3, 10);
#else
        int8_t  buf[4] = { (int8_t)this->q_.q0_, (int8_t)this->q_.q1_, (int8_t)this->q_.q2_, (int8_t)this->q_.q3_ };
        VCP_Tx((uint8_t*)buf, 4, 10);
#endif
//        printf("%d,%d,%d\r\n", (int)(euler_angle.v_[0]), (int)(euler_angle.v_[1]), (int)(euler_angle.v_[2]));
    }

    return euler_angle;
}

/* end of file ****************************************************************************************************** */
