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

#ifndef __SOC_UART_H__
#define __SOC_UART_H__

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

/* ****************************************************************************************************************** */

typedef enum _UartPort
{
    _UartPort_1,    // serial Rx
    _UartPort_2,    // comms.
    _UartPort_N
}
UartPort;

typedef void    *UartHandle;

typedef void    (*UartCallback)(int16_t len, bool error, void *param);

/* ****************************************************************************************************************** */

extern UartHandle   soc_uart_init(UartPort port, char *cfg_str, UartCallback cbf, void *param);
extern bool         soc_uart_tx(UartHandle h, uint8_t *data, int16_t len);
extern bool         soc_uart_rx(UartHandle h, uint8_t *data, int16_t len);
extern int16_t      soc_uart_tx_timeout(UartHandle h, uint8_t *data, int16_t len, uint32_t timeout_ms);
extern int16_t      soc_uart_rx_timeout(UartHandle h, uint8_t *data, int16_t len, uint32_t timeout_ms);

/* ****************************************************************************************************************** */

#endif /* __SOC_UART_H__ */

/* end of file ****************************************************************************************************** */
