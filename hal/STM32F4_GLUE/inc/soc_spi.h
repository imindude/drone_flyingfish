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

#ifndef __SOC_SPI_H__
#define __SOC_SPI_H__

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>
#include "soc_gpio.h"

/* ****************************************************************************************************************** */

typedef enum _SpiPort
{
    _SpiPort_1,
    _SpiPort_N
}
SpiPort;

typedef void    *SpiHandle;

typedef void    (*SpiCallback)(int16_t len, bool error, void *param);

/* ****************************************************************************************************************** */

extern SpiHandle    soc_spi_init(SpiPort port, GpioPin ncs);
extern bool         soc_spi_set_speed(SpiHandle h, uint32_t khz);
extern bool         soc_spi_tx(SpiHandle h, uint8_t *data, int16_t len, SpiCallback cbf, void *param);
extern bool         soc_spi_rx(SpiHandle h, uint8_t *data, int16_t len, SpiCallback cbf, void *param);
extern bool         soc_spi_txrx(SpiHandle h, uint8_t *tx, uint8_t *rx, int16_t len, SpiCallback cbf, void *param);
extern int16_t      soc_spi_tx_timeout(SpiHandle h, uint8_t *data, int16_t len, uint32_t timeout_ms);
extern int16_t      soc_spi_rx_timeout(SpiHandle h, uint8_t *data, int16_t len, uint32_t timeout_ms);
extern int16_t      soc_spi_txrx_timeout(SpiHandle h, uint8_t *tx, uint8_t *rx, int16_t len, uint32_t timeout_ms);
extern bool         soc_spi_lock(SpiHandle h);
extern void         soc_spi_unlock(SpiHandle h);

/* ****************************************************************************************************************** */

#endif /* __SOC_SPI_H__ */

/* end of file ****************************************************************************************************** */
