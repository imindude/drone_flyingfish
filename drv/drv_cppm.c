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

#include "drv_cppm.h"
#include "soc_pwm.h"

/* ****************************************************************************************************************** */

#define CPPM_MAX_CHANNEL        8
#define CPPM_SYNC_DELAY_US      2200
#define CPPM_FAILSAFE_US        4000

typedef struct _LocalData
{
    PwmInHandle     pwm_;
    bool            sync_;
    uint8_t         n_recv_;
    int16_t         *channels_;
    CppmCallback    callback_;
    void            *param_;
} LocalData;

/* ****************************************************************************************************************** */

static LocalData    cppm_data[_CppmPort_N] = {

        [_CppmPort_1] = {

                .pwm_      = NULL,
                .sync_     = false,
                .n_recv_   = 0,
                .channels_ = NULL,
                .callback_ = NULL,
                .param_    = NULL
        }
};

/* ****************************************************************************************************************** */

static void _pwmin_callback(uint32_t duration_us, void *param)
{
    LocalData   *this = (LocalData*)param;

    if (duration_us > CPPM_FAILSAFE_US)
    {
        this->n_recv_ = 0;
        if (this->callback_)
            this->callback_(this->n_recv_, true, this->param_);
    }
    else if (duration_us >= CPPM_SYNC_DELAY_US)
    {
        this->n_recv_ = 0;
        this->sync_   = true;
    }
    else if (this->sync_)
    {
        this->channels_[this->n_recv_] = (int16_t)duration_us;
        this->n_recv_++;

        if (this->n_recv_ >= CPPM_MAX_CHANNEL)
        {
            if (this->callback_)
                this->callback_(this->n_recv_, false, this->param_);

            this->n_recv_ = 0;
            this->sync_   = false;
        }
    }
}

CppmHandle drv_cppm_init(CppmPort port, int16_t *channels, CppmCallback cbf, void *param)
{
    if (port < _CppmPort_N)
    {
        cppm_data[port].channels_ = channels;
        cppm_data[port].callback_ = cbf;
        cppm_data[port].param_    = param;

        cppm_data[port].pwm_ = soc_pwmin_init(_PwmInPin_1, _pwmin_callback, &cppm_data[port]);

        return (CppmHandle)(&cppm_data[port]);
    }

    return NULL;
}

/* end of file ****************************************************************************************************** */
