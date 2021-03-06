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

#ifndef __DRV_CPPM_H__
#define __DRV_CPPM_H__

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>

/* ****************************************************************************************************************** */

typedef enum _CppmPort {

    _CppmPort_1,
    _CppmPort_N
}
CppmPort;

typedef void    (*CppmCallback)(int8_t n_ch, bool failsafe, void *param);

typedef void    *CppmHandle;

/* ****************************************************************************************************************** */

extern CppmHandle   drv_cppm_init(CppmPort port, int16_t *channels, CppmCallback cbf, void *param);

/* ****************************************************************************************************************** */

#endif /* __DRV_CPPM_H__ */

/* end of file ****************************************************************************************************** */
