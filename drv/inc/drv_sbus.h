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

#ifndef __DRV_SBUS_H__
#define __DRV_SBUS_H__

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>

/* ****************************************************************************************************************** */

typedef enum _SbusPort {

    _SbusPort_1,
    _SbusPort_N
}
SbusPort;

typedef void    (*SbusCallback)(int8_t n_ch, bool failsafe, void *param);

typedef void    *SbusHandle;

/* ****************************************************************************************************************** */

extern SbusHandle   drv_sbus_init(SbusPort port, int16_t *channels, SbusCallback cbf, void *param);

/* ****************************************************************************************************************** */

#endif  // __DRV_SBUS_H__

/* end of file ****************************************************************************************************** */
