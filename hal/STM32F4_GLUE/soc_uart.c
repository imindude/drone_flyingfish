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

#include <stdlib.h>
#include "soc_uart.h"

/* ****************************************************************************************************************** */

extern UART_HandleTypeDef   huart1;
extern UART_HandleTypeDef   huart6;
extern DMA_HandleTypeDef    hdma_usart1_tx;
extern DMA_HandleTypeDef    hdma_usart1_rx;

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    UART_HandleTypeDef  *uart_;
    DMA_HandleTypeDef   *dma_tx_;
    DMA_HandleTypeDef   *dma_rx_;
    UartCallback        callback_;
    void                *param_;
}
LocalData;

/* ****************************************************************************************************************** */

static LocalData    uart_data[_UartPort_N] = {

        [_UartPort_1] = {

                .uart_     = &huart6,
                .dma_tx_   = NULL,
                .dma_rx_   = NULL,
                .callback_ = NULL,
                .param_    = NULL
        },
        [_UartPort_2] = {

                .uart_     = &huart1,
                .dma_tx_   = &hdma_usart1_tx,
                .dma_rx_   = &hdma_usart1_rx,
                .callback_ = NULL,
                .param_    = NULL
        }
};

/* ****************************************************************************************************************** */

UartHandle soc_uart_init(UartPort port, char *cfg_str, UartCallback cbf, void *param)
{
    if (port < _UartPort_N)
    {
        /**
         * "115200:8N1:X"
         */

        char    *p, *q;
        int8_t  word_len;

        // get baudrate "115200"
        uart_data[port].uart_->Init.BaudRate = strtoul(cfg_str, &q, 10);

        // skip ':'
        p = q + 1;

        // get databits "8"
        word_len = strtoul(p, &q, 10);

        // get parity bit "N"
        p = q;
        switch (*p)
        {
        default:
        case 'N':
            uart_data[port].uart_->Init.Parity = UART_PARITY_NONE;
            break;
        case 'E':
            uart_data[port].uart_->Init.Parity = UART_PARITY_EVEN;
            word_len++;
            break;
        case 'O':
            uart_data[port].uart_->Init.Parity = UART_PARITY_ODD;
            word_len++;
            break;
        }

        // set word length
        if (word_len == 9)
            uart_data[port].uart_->Init.WordLength = UART_WORDLENGTH_9B;
        else
            uart_data[port].uart_->Init.WordLength = UART_WORDLENGTH_8B;

        // get stop bit "1"
        p++;
        if (*p == 2)
            uart_data[port].uart_->Init.StopBits = UART_STOPBITS_2;
        else
            uart_data[port].uart_->Init.StopBits = UART_STOPBITS_1;

        // skip ':'
        p++;

        // get direction "X"
        p++;
        switch (*p)
        {
        default:
        case 'X':
            uart_data[port].uart_->Init.Mode = UART_MODE_TX_RX;
            break;
        case 'T':
            uart_data[port].uart_->Init.Mode = UART_MODE_TX;
            break;
        case 'R':
            uart_data[port].uart_->Init.Mode = UART_MODE_RX;
            break;
        }

        if (HAL_UART_Init(uart_data[port].uart_) == HAL_OK)
        {
            uart_data[port].callback_ = cbf;
            uart_data[port].param_    = param;

            return (UartHandle)(&uart_data[port]);
        }
    }

    return NULL;
}

bool soc_uart_tx(UartHandle h, uint8_t *data, int16_t len)
{
    LocalData   *this = (LocalData*)h;
    HAL_StatusTypeDef   st;

    if (this->dma_tx_)
        st = HAL_UART_Transmit_DMA(this->uart_, data, len);
    else
        st = HAL_UART_Transmit_IT(this->uart_, data, len);

    return (st == HAL_OK) ? true : false;
}

bool soc_uart_rx(UartHandle h, uint8_t *data, int16_t len)
{
    LocalData   *this = (LocalData*)h;
    HAL_StatusTypeDef   st;

    if (this->dma_rx_)
        st = HAL_UART_Receive_DMA(this->uart_, data, len);
    else
        st = HAL_UART_Receive_IT(this->uart_, data, len);

    return (st == HAL_OK) ? true : false;
}

int16_t soc_uart_tx_timeout(UartHandle h, uint8_t *data, int16_t len, uint32_t timeout_ms)
{
    LocalData   *this = (LocalData*)h;

    if (HAL_UART_Transmit(this->uart_, data, len, timeout_ms) != HAL_OK)
        return this->uart_->TxXferSize - this->uart_->TxXferCount - 1;

    return len;
}

int16_t soc_uart_rx_timeout(UartHandle h, uint8_t *data, int16_t len, uint32_t timeout_ms)
{
    LocalData   *this = (LocalData*)h;

    if (HAL_UART_Receive(this->uart_, data, len, timeout_ms) != HAL_OK)
        return this->uart_->RxXferSize - this->uart_->RxXferCount - 1;

    return len;
}

/* ****************************************************************************************************************** */

static void find_callback(UART_HandleTypeDef *huart, int16_t len, bool error)
{
    for (int8_t i = 0; i < _UartPort_N; i++)
    {
        if (uart_data[i].uart_ == huart)
        {
            if (uart_data[i].callback_)
                uart_data[i].callback_(len, error, uart_data[i].param_);
            break;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    find_callback(huart, huart->TxXferSize, false);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    find_callback(huart, huart->RxXferSize, false);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    find_callback(huart, 0, true);
}

/* ****************************************************************************************************************** */


/* end of file ****************************************************************************************************** */
