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

/* ****************************************************************************************************************** */

#include "drv_sbus.h"
#include "soc_uart.h"

/* ****************************************************************************************************************** */

#define SBUS_PACKET_LEN         25
#define SBUS_MAX_CHANNEL        18
#define SBUS_SYNC_DELAY_US      3000
#define SBUS_FAILSAFE_US        100000
#define SBUS_SYNC_BYTE          0x0F
#define SBUS_MIN_VALUE          172
#define SBUS_MAX_VALUE          1812

#pragma pack(push, 1)
typedef union _Packet
{
    struct {
        /**
         * SYNC(1byte) + DATA(22byte) +
         */
        uint8_t     sync_;

        uint32_t    ch1_ : 11;
        uint32_t    ch2_ : 11;
        uint32_t    ch3_ : 11;
        uint32_t    ch4_ : 11;
        uint32_t    ch5_ : 11;
        uint32_t    ch6_ : 11;
        uint32_t    ch7_ : 11;
        uint32_t    ch8_ : 11;
        uint32_t    ch9_ : 11;
        uint32_t    ch10_ : 11;
        uint32_t    ch11_ : 11;
        uint32_t    ch12_ : 11;
        uint32_t    ch13_ : 11;
        uint32_t    ch14_ : 11;
        uint32_t    ch15_ : 11;
        uint32_t    ch16_ : 11;

        uint8_t     flag_;
        uint8_t     end_;
    } frame_;
    uint8_t     bytes_[SBUS_PACKET_LEN];
} Packet;
#pragma pack(pop)

typedef struct LocalData
{
    UartHandle      uart_;
    Packet          packet_;
    int16_t         *channels_;
    bool            sync_;
    uint32_t        last_us_;
    SbusCallback    callback_;
    void            *param_;
} LocalData;

/* ****************************************************************************************************************** */

static LocalData    sbus_data[_SbusPort_N] = {

        [_SbusPort_1] = {

                .uart_     = NULL,
                .channels_ = NULL,
                .sync_     = false,
                .last_us_  = 0,
                .callback_ = NULL,
                .param_    = NULL
        }
};

/* ****************************************************************************************************************** */

static void sbus_callback(LocalData *this, bool failsafe)
{
#define FLAG_CH17       0b0001
#define FLAG_CH18       0b0010
#define FLAG_SIG_LOSS   0b0100
#define FLAG_FAILSAFE   0b1000

    if (this->callback_) {

        this->channels_[ 0] = this->packet_.frame_.ch1_;
        this->channels_[ 1] = this->packet_.frame_.ch2_;
        this->channels_[ 2] = this->packet_.frame_.ch3_;
        this->channels_[ 3] = this->packet_.frame_.ch4_;
        this->channels_[ 4] = this->packet_.frame_.ch5_;
        this->channels_[ 5] = this->packet_.frame_.ch6_;
        this->channels_[ 6] = this->packet_.frame_.ch7_;
        this->channels_[ 7] = this->packet_.frame_.ch8_;
        this->channels_[ 8] = this->packet_.frame_.ch9_;
        this->channels_[ 9] = this->packet_.frame_.ch10_;
        this->channels_[10] = this->packet_.frame_.ch11_;
        this->channels_[11] = this->packet_.frame_.ch12_;
        this->channels_[12] = this->packet_.frame_.ch13_;
        this->channels_[13] = this->packet_.frame_.ch14_;
        this->channels_[14] = this->packet_.frame_.ch15_;
        this->channels_[15] = this->packet_.frame_.ch16_;
        this->channels_[16] = this->packet_.frame_.flag_ & FLAG_CH17 ? SBUS_MAX_VALUE : SBUS_MIN_VALUE;
        this->channels_[17] = this->packet_.frame_.flag_ & FLAG_CH18 ? SBUS_MAX_VALUE : SBUS_MIN_VALUE;

        if (this->packet_.frame_.flag_ & (FLAG_FAILSAFE | FLAG_SIG_LOSS))
            failsafe = true;

        this->callback_(SBUS_MAX_CHANNEL, failsafe, this->param_);
    }

#undef FLAG_CH17
#undef FLAG_CH18
#undef FLAG_SIG_LOSS
#undef FLAG_FAILSAFE
}

static void _uart_callback(int16_t size, bool error, void *param)
{
    LocalData   *this = (LocalData*)param;

    if (error || (size == 0))
    {
        this->sync_ = false;
        this->packet_.bytes_[0] = 0;
        soc_uart_rx(this->uart_, this->packet_.bytes_, 1);
    }
    else
    {
        uint32_t    now_us = sys_get_micros();
        uint32_t    dt_us = now_us - this->last_us_;

        if (dt_us < SBUS_FAILSAFE_US) {

            if (this->sync_)
            {
                if (this->packet_.bytes_[0] == SBUS_SYNC_BYTE)
                {
                    sbus_callback(this, false);

                    this->packet_.bytes_[0] = 0;
                    soc_uart_rx(this->uart_, this->packet_.bytes_, SBUS_PACKET_LEN);
                }
                else
                {
                    this->sync_ = false;
                    this->packet_.bytes_[0] = 0;
                    soc_uart_rx(this->uart_, this->packet_.bytes_, 1);
                }
            }
            else
            {
                if ((dt_us > SBUS_SYNC_DELAY_US) && (this->packet_.bytes_[0] == SBUS_SYNC_BYTE))
                {
                    this->sync_ = true;
                    soc_uart_rx(this->uart_, this->packet_.bytes_ + 1, SBUS_PACKET_LEN - 1);
                }
                else
                {
                    this->packet_.bytes_[0] = 0;
                    soc_uart_rx(this->uart_, this->packet_.bytes_, 1);
                }
            }
        }
        else
        {
            sbus_callback(this, true);

            this->sync_ = false;
            this->packet_.bytes_[0] = 0;
            soc_uart_rx(this->uart_, this->packet_.bytes_, 1);
        }

        this->last_us_ = now_us;
    }
}

SbusHandle drv_sbus_init(SbusPort port, int16_t *channels, SbusCallback cbf, void *param)
{
    if (port < _SbusPort_N)
    {
        sbus_data[port].channels_ = channels;
        sbus_data[port].last_us_  = sys_get_micros();
        sbus_data[port].callback_ = cbf;
        sbus_data[port].param_    = param;

        sbus_data[port].uart_ = soc_uart_init(_UartPort_1, "100000:8E2:R", _uart_callback, &sbus_data[port]);
        soc_uart_rx(sbus_data[port].uart_, sbus_data[port].packet_.bytes_, 1);

        return (SbusHandle)(&sbus_data[port]);
    }

    return NULL;
}

/* end of file ****************************************************************************************************** */
