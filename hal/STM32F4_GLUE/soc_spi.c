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

#include "soc_spi.h"

/* ****************************************************************************************************************** */

extern SPI_HandleTypeDef    hspi3;
extern DMA_HandleTypeDef    hdma_spi3_tx;
extern DMA_HandleTypeDef    hdma_spi3_rx;

/* ****************************************************************************************************************** */

typedef struct _LocalData
{
    SPI_HandleTypeDef   *spi_;
    DMA_HandleTypeDef   *dma_tx_;
    DMA_HandleTypeDef   *dma_rx_;
    bool                locked_;
    GpioHandle          ncs_;
    SpiCallback         callback_;
    void                *param_;
} LocalData;

/* ****************************************************************************************************************** */

static LocalData    spi_data[_SpiPort_N] = {

        [_SpiPort_1] = {

                .spi_      = &hspi3,
                .dma_tx_   = &hdma_spi3_tx,
                .dma_rx_   = &hdma_spi3_rx,
                .locked_   = false,
                .ncs_      = NULL,
                .callback_ = NULL,
                .param_    = NULL
        }
};

/* ****************************************************************************************************************** */

SpiHandle soc_spi_init(SpiPort port, GpioPin ncs)
{
    if (port < _SpiPort_N)
    {
        if (spi_data[port].spi_->State == HAL_SPI_STATE_RESET)
        {
            if (HAL_SPI_Init(spi_data[port].spi_) == HAL_OK)
            {
                spi_data[port].ncs_ = soc_gpio_init(ncs);

                return (SpiHandle)(&spi_data[port]);
            }
        }
    }

    return NULL;
}

bool soc_spi_set_speed(SpiHandle h, uint32_t khz)
{
    LocalData   *this = (LocalData*)h;
    int16_t     prescalar;
    int32_t     freq_khz;

    // SPI3 is on the APB1 - 84MHz
    freq_khz = HAL_RCC_GetPCLK1Freq() / 1000;

    for (prescalar = 1; prescalar <= 8; prescalar++)
    {
        if ((freq_khz / (1 << prescalar)) <= khz)
            break;
    }

    switch (1 << prescalar)
    {
    case 2:
        this->spi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        break;
    case 4:
        this->spi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
        break;
    case 8:
        this->spi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        break;
    case 16:
        this->spi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        break;
    case 32:
        this->spi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
        break;
    case 64:
        this->spi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        break;
    case 128:
        this->spi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
        break;
    default:
    case 256:
        this->spi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
        break;
    }

    if (HAL_SPI_Init(this->spi_) == HAL_OK)
        return true;

    return false;
}

bool soc_spi_tx(SpiHandle h, uint8_t *data, int16_t len, SpiCallback cbf, void *param)
{
    LocalData   *this = (LocalData*)h;
    HAL_StatusTypeDef   st;

    this->callback_ = cbf;
    this->param_    = param;

    if (this->dma_tx_)
        st = HAL_SPI_Transmit_DMA(this->spi_, data, len);
    else
        st = HAL_SPI_Transmit_IT(this->spi_, data, len);

    if (st == HAL_OK)
        return true;

    this->callback_ = NULL;
    this->param_    = NULL;

    return false;
}

bool soc_spi_rx(SpiHandle h, uint8_t *data, int16_t len, SpiCallback cbf, void *param)
{
    LocalData   *this = (LocalData*)h;
    HAL_StatusTypeDef   st;

    this->callback_ = cbf;
    this->param_    = param;

    if (this->dma_rx_)
        st = HAL_SPI_Receive_DMA(this->spi_, data, len);
    else
        st = HAL_SPI_Receive_IT(this->spi_, data, len);

    if (st == HAL_OK)
        return true;

    this->callback_ = NULL;
    this->param_    = NULL;

    return false;
}

bool soc_spi_txrx(SpiHandle h, uint8_t *tx, uint8_t *rx, int16_t len, SpiCallback cbf, void *param)
{
    LocalData   *this = (LocalData*)h;
    HAL_StatusTypeDef   st;

    this->callback_ = cbf;
    this->param_    = param;

    if (this->dma_rx_)
        st = HAL_SPI_TransmitReceive_DMA(this->spi_, tx, rx, len);
    else
        st = HAL_SPI_TransmitReceive_IT(this->spi_, tx, rx, len);

    if (st == HAL_OK)
        return true;

    this->callback_ = NULL;
    this->param_    = NULL;

    return false;
}

int16_t soc_spi_tx_timeout(SpiHandle h, uint8_t *data, int16_t len, uint32_t timeout_ms)
{
    LocalData   *this = (LocalData*)h;

    if (HAL_SPI_Transmit(this->spi_, data, len, timeout_ms) != HAL_OK)
        return this->spi_->TxXferSize - this->spi_->TxXferCount - 1;

    return len;
}

int16_t soc_spi_rx_timeout(SpiHandle h, uint8_t *data, int16_t len, uint32_t timeout_ms)
{
    LocalData   *this = (LocalData*)h;

    if (HAL_SPI_Receive(this->spi_, data, len, timeout_ms) != HAL_OK)
        return this->spi_->RxXferSize - this->spi_->RxXferCount - 1;

    return len;
}

int16_t soc_spi_txrx_timeout(SpiHandle h, uint8_t *tx, uint8_t *rx, int16_t len, uint32_t timeout_ms)
{
    LocalData   *this = (LocalData*)h;

    if (HAL_SPI_TransmitReceive(this->spi_, tx, rx, len, timeout_ms) != HAL_OK)
        return this->spi_->RxXferSize - this->spi_->RxXferCount - 1;

    return len;
}

bool soc_spi_lock(SpiHandle h)
{
    LocalData   *this = (LocalData*)h;

    if (this->locked_)
        return false;

    this->locked_ = true;
    soc_gpio_set_low(this->ncs_);

    return true;
}

void soc_spi_unlock(SpiHandle h)
{
    LocalData   *this = (LocalData*)h;

    soc_gpio_set_high(this->ncs_);
    this->locked_ = false;
}

/* ****************************************************************************************************************** */

static void find_callback(SPI_HandleTypeDef *hspi, int16_t len, bool error) {

    for (int8_t i = 0; i < _SpiPort_N; i++)
    {
        if (spi_data[i].spi_ == hspi) {

            if (spi_data[i].callback_)
                spi_data[i].callback_(len, error, spi_data[i].param_);
            break;
        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    find_callback(hspi, hspi->TxXferSize, false);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    find_callback(hspi, hspi->RxXferSize, false);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    find_callback(hspi, hspi->RxXferSize, false);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    find_callback(hspi, 0, true);
}

/* end of file ****************************************************************************************************** */
