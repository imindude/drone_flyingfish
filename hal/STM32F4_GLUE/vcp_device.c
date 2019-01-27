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

#include "vcp_device.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "stm32f4xx.h"

/* ****************************************************************************************************************** */

USBD_HandleTypeDef  husbd;

/* ****************************************************************************************************************** */

void VCP_Init(void)
{
    USBD_Init(&husbd, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&husbd, &USBD_CDC);
    USBD_CDC_RegisterInterface(&husbd, &USBD_Interface_fops_FS);
    USBD_Start(&husbd);
}

int16_t VCP_Tx(uint8_t *buffer, int16_t size, uint32_t timeout_ms)
{
    (void)timeout_ms;
    return (CDC_Transmit_FS(buffer, size) == USBD_OK) ? size : 0;
}

int16_t VCP_Rx(uint8_t *buffer, int16_t size, uint32_t timeout_ms)
{
    int16_t n_rx = 0;
    int16_t tmp;

    while (timeout_ms > 0)
    {
        tmp = CDC_ReadRx(buffer + n_rx, size - n_rx);
        n_rx += tmp;

        if (n_rx >= size)
            break;

        timeout_ms--;

        HAL_Delay(1);
    }

    return n_rx;
}

int _write(int fd, char *str, int len)
{
    (void)fd;

    return VCP_Tx((uint8_t*)str, len, 10);
}

/* end of file ****************************************************************************************************** */
