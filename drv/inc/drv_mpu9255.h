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

#ifndef __DRV_MPU9255_H__
#define __DRV_MPU9255_H__

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>

/* ****************************************************************************************************************** */

typedef enum _MpuDev
{
    _MpuDev_1,
    _MpuDev_N
}
MpuDev;

typedef enum _MpuMode
{
    _MpuMode_1K,
    _MpuMode_8K,
    _MpuMode_N
}
MpuMode;

typedef void    *MpuHandle;

/* ****************************************************************************************************************** */

extern MpuHandle    drv_mpu_init(MpuDev dev, MpuMode mode);
extern bool         drv_mpu_is_ready(MpuHandle h, uint32_t now_us);
extern void         drv_mpu_read_data(MpuHandle h, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az);
extern float        drv_mpu_get_gyro_factor(MpuHandle h);
extern float        drv_mpu_get_acc_factor(MpuHandle h);
extern float        drv_mpu_get_gyro_zero_tolerance(MpuHandle t);

/* ****************************************************************************************************************** */

#endif /* __DRV_MPU9255_H__ */

/* end of file ****************************************************************************************************** */
