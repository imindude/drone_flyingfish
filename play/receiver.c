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

#include "receiver.h"
#include "drv_cppm.h"
#include "drv_sbus.h"

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    Handle              receiver_;
    StickMap            stick_map_;
    int16_t             key_buff_[MAX_RC_CHANNELS];
    float               key_data_[MAX_RC_CHANNELS];
    int8_t              n_channel_;
    bool                failsafe_;
    ReceiverCallback    callback_;
    void                *param_;
}
LocalData;

/* ****************************************************************************************************************** */

static void _cppm_callback(int8_t n_ch, bool failsafe, void *param)
{
    LocalData   *this = (LocalData*)param;

    this->n_channel_ = n_ch;
    this->failsafe_  = failsafe;

    for (int8_t i = 0; i < this->n_channel_; i++)
    {
        // https://www.wolframalpha.com/input/?i=linear+fit+%7B1000,+-1%7D,+%7B2000,+1%7D,+%7B1500,+0%7D
        this->key_data_[i] = 0.002f * this->key_buff_[i] - 3.0f;
    }

    if (this->callback_)
        this->callback_(this->n_channel_, this->failsafe_, this->param_);
}

static void _sbus_callback(int8_t n_ch, bool failsafe, void *param)
{
    LocalData   *this = (LocalData*)param;

    this->n_channel_ = n_ch;
    this->failsafe_  = failsafe;

    for (int8_t i = 0; i < this->n_channel_; i++)
    {
        // [FrSky(X4RSB)] https://www.wolframalpha.com/input/?i=linear+fit+%7B172,+-1%7D,+%7B1811,+1%7D,+%7B992,+0%7D
        this->key_data_[i] = 0.00122026f * this->key_buff_[i] - 1.21009f;
    }

    if (this->callback_)
        this->callback_(this->n_channel_, this->failsafe_, this->param_);
}

ReceiverHandle receiver_init(ReceiverType receiver, StickMap map, ReceiverCallback cbf, void *param)
{
    if (receiver < _ReceiverType_N)
    {
        LocalData   *this = (LocalData*)calloc(1, sizeof(LocalData));

        this->stick_map_ = map;
        this->callback_  = cbf;
        this->param_     = param;

        if (receiver == _ReceiverType_CPPM)
            this->receiver_ = drv_cppm_init(_CppmPort_1, this->key_buff_, _cppm_callback, this);
        else
            this->receiver_ = drv_sbus_init(_SbusPort_1, this->key_buff_, _sbus_callback, this);

        return (ReceiverHandle)this;
    }

    return NULL;
}

float receiver_get_sticks(ReceiverHandle h, EulerF32 *rpy)
{
    LocalData   *this = (LocalData*)h;

    rpy->rpy_.r_ = this->key_data_[this->stick_map_.rol_];
    rpy->rpy_.p_ = this->key_data_[this->stick_map_.pit_];
    rpy->rpy_.y_ = this->key_data_[this->stick_map_.yaw_];

    return this->key_data_[this->stick_map_.thr_];
}

float receiver_get_key(ReceiverHandle h, uint8_t ch)
{
    LocalData   *this = (LocalData*)h;
    return this->key_data_[ch];
}

/* end of file ****************************************************************************************************** */
