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

#include "drv_mpu9255.h"
#include "soc_spi.h"

/* ****************************************************************************************************************** */

#define MPU9255_READ_LEN            14      // AXH|AXL|AYH|AYL|AZH|AZL|TH|TL|GXH|GXL|GYH|GYL|GZH|GZL
#define MPU9255_SPI_SPEED_KHZ_INIT  1000
#define MPU9255_SPI_SPEED_KHZ_READ  21500   // optimised speed (recommended speed is 20MHz)
#define MPU9255_WHOAMI              0x73
#define MPU9255_ZERO_TOLERANCE_DPS  15      // MPU9255's gyroscope ZERO tolerance in 25°C is ±5

/**
 * MPU9250 register command
 */

#define MPUREG_RD(r)        (r | 0x80)
#define MPUREG_WR(r)        (r)

#define PWR_MGMT_1_H_RESET              (1 << 7)
#define PWR_MGMT_1_CLKSEL_AUTO          3

#define PWR_MGMT_2_GYRO_ACCEL_EN        0

#define SIGNAL_PATH_RESET_GYRO          (1 << 2)
#define SIGNAL_PATH_RESET_ACCEL         (1 << 1)
#define SIGNAL_PATH_RESET_TEMP          (1 << 0)

#define USER_CTRL_I2C_MST_EN            (1 << 5)
#define USER_CTRL_I2C_IF_DIS            (1 << 4)
#define USER_CTRL_FIFO_RST              (1 << 2)
#define USER_CTRL_I2C_MST_RST           (1 << 1)
#define USER_CTRL_SIG_COND_RST          (1 << 0)

#define SMPLRT_DIV_1                    0
#define SMPLRT_DIV_4                    3

#define CONFIG_DLPF_250HZ_970US         0   // 8K
#define CONFIG_DLPF_184HZ_2900US        1   // 1K
#define CONFIG_DLPF_3600HZ_170US        7   // 32K

#define GYRO_CONFIG_FS_SEL_1000DPS      (0b10 << 3)
#define GYRO_CONFIG_FS_SEL_2000DPS      (0b11 << 3)
#define GYRO_CONFIG_FCHOICE_B_1K        0b00
#define GYRO_CONFIG_FCHOICE_B_8K        0b00

#define ACCEL_CONFIG_FS_SEL_4G          (0b01 << 3)
#define ACCEL_CONFIG_FS_SEL_8G          (0b10 << 3)

#define ACCEL_CONFIG2_FCHOICE_B_1K      (0 << 3)
#define ACCEL_CONFIG2_FCHOICE_B_4K      (1 << 3)
#define ACCEL_CONFIG2_DLPF_1046HZ_503US 0   // 4K
#define ACCEL_CONFIG2_DLPF_218HZ_1880US 1   // 1K

#define INT_PIN_CFG_LATCH_INT_EN        (1 << 5)
#define INT_PIN_CFG_ANYRD_2CLEAR        (1 << 4)

#define INT_STATUS_RAW_DATA_RDY_INT     (1 << 0)

#define INT_ENABLE_RAW_DRY_EN           (1 << 0)

#define I2C_MST_CTRL_400KHZ             13
#define I2C_MST_CTRL_500KHZ             9

#define I2C_SLVx_EN                     (1 << 7)
#define I2C_SLVx_LEN(x)                 ((x) & 0x0F)

#define I2C_SLV0_DLY_EN                 (1 << 0)

typedef enum _Regs
{
    _Regs_SELF_TEST_X_GYRO   = 0x00,
    _Regs_SELF_TEST_Y_GYRO   = 0x01,
    _Regs_SELF_TEST_Z_GYRO   = 0x02,
    _Regs_SELF_TEST_X_ACCEL  = 0x0D,
    _Regs_SELF_TEST_Y_ACCEL  = 0x0E,
    _Regs_SELF_TEST_Z_ACCEL  = 0x0F,
    _Regs_XG_OFFSET_H        = 0x13,
    _Regs_XG_OFFSET_L        = 0x14,
    _Regs_YG_OFFSET_H        = 0x15,
    _Regs_YG_OFFSET_L        = 0x16,
    _Regs_ZG_OFFSET_H        = 0x17,
    _Regs_ZG_OFFSET_L        = 0x18,
    _Regs_SMPLRT_DIV         = 0x19,
    _Regs_CONFIG             = 0x1A,
    _Regs_GYRO_CONFIG        = 0x1B,
    _Regs_ACCEL_CONFIG       = 0x1C,
    _Regs_ACCEL_CONFIG2      = 0x1D,
    _Regs_LP_ACCEL_ODR       = 0x1E,
    _Regs_WOM_THR            = 0x1F,
    _Regs_FIFO_EN            = 0x23,
    _Regs_I2C_MST_CTRL       = 0x24,
    _Regs_I2C_SLV0_ADDR      = 0x25,
    _Regs_I2C_SLV0_REG       = 0x26,
    _Regs_I2C_SLV0_CTRL      = 0x27,
    _Regs_I2C_SLV1_ADDR      = 0x28,
    _Regs_I2C_SLV1_REG       = 0x29,
    _Regs_I2C_SLV1_CTRL      = 0x2A,
    _Regs_I2C_SLV2_ADDR      = 0x2B,
    _Regs_I2C_SLV2_REG       = 0x2C,
    _Regs_I2C_SLV2_CTRL      = 0x2D,
    _Regs_I2C_SLV3_ADDR      = 0x2E,
    _Regs_I2C_SLV3_REG       = 0x2F,
    _Regs_I2C_SLV3_CTRL      = 0x30,
    _Regs_I2C_SLV4_ADDR      = 0x31,
    _Regs_I2C_SLV4_REG       = 0x32,
    _Regs_I2C_SLV4_DO        = 0x33,
    _Regs_I2C_SLV4_CTRL      = 0x34,
    _Regs_I2C_SLV4_DI        = 0x35,
    _Regs_I2C_MST_STATUS     = 0x36,
    _Regs_INT_PIN_CFG        = 0x37,
    _Regs_INT_ENABLE         = 0x38,
    _Regs_INT_STATUS         = 0x3A,
    _Regs_ACCEL_XOUT_H       = 0x3B,
    _Regs_ACCEL_XOUT_L       = 0x3C,
    _Regs_ACCEL_YOUT_H       = 0x3D,
    _Regs_ACCEL_YOUT_L       = 0x3E,
    _Regs_ACCEL_ZOUT_H       = 0x3F,
    _Regs_ACCEL_ZOUT_L       = 0x40,
    _Regs_TEMP_OUT_H         = 0x41,
    _Regs_TEMP_OUT_L         = 0x42,
    _Regs_GYRO_XOUT_H        = 0x43,
    _Regs_GYRO_XOUT_L        = 0x44,
    _Regs_GYRO_YOUT_H        = 0x45,
    _Regs_GYRO_YOUT_L        = 0x46,
    _Regs_GYRO_ZOUT_H        = 0x47,
    _Regs_GYRO_ZOUT_L        = 0x48,
    _Regs_EXT_SENS_DATA_00   = 0x49,
    _Regs_EXT_SENS_DATA_01   = 0x4A,
    _Regs_EXT_SENS_DATA_02   = 0x4B,
    _Regs_EXT_SENS_DATA_03   = 0x4C,
    _Regs_EXT_SENS_DATA_04   = 0x4D,
    _Regs_EXT_SENS_DATA_05   = 0x4E,
    _Regs_EXT_SENS_DATA_06   = 0x4F,
    _Regs_EXT_SENS_DATA_07   = 0x50,
    _Regs_EXT_SENS_DATA_08   = 0x51,
    _Regs_EXT_SENS_DATA_09   = 0x52,
    _Regs_EXT_SENS_DATA_10   = 0x53,
    _Regs_EXT_SENS_DATA_11   = 0x54,
    _Regs_EXT_SENS_DATA_12   = 0x55,
    _Regs_EXT_SENS_DATA_13   = 0x56,
    _Regs_EXT_SENS_DATA_14   = 0x57,
    _Regs_EXT_SENS_DATA_15   = 0x58,
    _Regs_EXT_SENS_DATA_16   = 0x59,
    _Regs_EXT_SENS_DATA_17   = 0x5A,
    _Regs_EXT_SENS_DATA_18   = 0x5B,
    _Regs_EXT_SENS_DATA_19   = 0x5C,
    _Regs_EXT_SENS_DATA_20   = 0x5D,
    _Regs_EXT_SENS_DATA_21   = 0x5E,
    _Regs_EXT_SENS_DATA_22   = 0x5F,
    _Regs_EXT_SENS_DATA_23   = 0x60,
    _Regs_I2C_SLV0_DO        = 0x63,
    _Regs_I2C_SLV1_DO        = 0x64,
    _Regs_I2C_SLV2_DO        = 0x65,
    _Regs_I2C_SLV3_DO        = 0x66,
    _Regs_I2C_MST_DELAY_CTRL = 0x67,
    _Regs_SIGNAL_PATH_RESET  = 0x68,
    _Regs_MOT_DETECT_CTRL    = 0x69,
    _Regs_USER_CTRL          = 0x6A,
    _Regs_PWR_MGMT_1         = 0x6B,
    _Regs_PWR_MGMT_2         = 0x6C,
    _Regs_FIFO_COUNTH        = 0x72,
    _Regs_FIFO_COUNTL        = 0x73,
    _Regs_FIFO_R_W           = 0x74,
    _Regs_WHO_AM_I           = 0x75,
    _Regs_XA_OFFSET_H        = 0x77,
    _Regs_XA_OFFSET_L        = 0x78,
    _Regs_YA_OFFSET_H        = 0x7A,
    _Regs_YA_OFFSET_L        = 0x7B,
    _Regs_ZA_OFFSET_H        = 0x7D,
    _Regs_ZA_OFFSET_L        = 0x7E,
    _Regs_End
}
Regs;

typedef enum _Status
{
    _Status_Waiting,
    _Status_Reading,
    _Status_Done
}
Status;

typedef struct _LocalData
{
    SpiHandle   spi_;
    Status      status_;
    uint32_t    last_read_us_;
    uint16_t    read_term_us_;
    uint8_t     read_buffer_[1 + MPU9255_READ_LEN];
    float       gyro_factor_;
    float       acc_factor_;
}
LocalData;

/* ****************************************************************************************************************** */

static LocalData    mpu_data[_MpuDev_N] = {

        [_MpuDev_1] = {

                .spi_          = NULL,
                .status_       = _Status_Waiting,
                .last_read_us_ = 0,
                .read_term_us_ = 0,
                .gyro_factor_  = 0.0f,
                .acc_factor_   = 0.0f
        }
};

/* ****************************************************************************************************************** */

static void _spi_callback(int16_t len, bool error, void *param)
{
    (void)len;
    (void)error;

    LocalData   *this = (LocalData*)param;

    soc_spi_unlock(this->spi_);
    this->status_ = _Status_Done;
}

static void write_reg(LocalData *this, uint8_t reg, uint8_t val)
{
    uint8_t data[2] = { MPUREG_WR(reg), val };

    soc_spi_lock(this->spi_);
    soc_spi_tx_timeout(this->spi_, data, 2, 10);
    soc_spi_unlock(this->spi_);
}

static uint8_t read_reg(LocalData *this, uint8_t reg)
{
    uint8_t data[2] = { MPUREG_RD(reg), 0 };

    soc_spi_lock(this->spi_);
    soc_spi_txrx_timeout(this->spi_, data, data, 2, 10);
    soc_spi_unlock(this->spi_);

    return data[1];
}

static void init_reg(LocalData *this, MpuMode mode)
{
    uint8_t     config, gyro_cfg, acc_cfg, acc_cfg2;

    if (mode == _MpuMode_8K)
    {
        config   = CONFIG_DLPF_250HZ_970US;
        gyro_cfg = GYRO_CONFIG_FS_SEL_2000DPS | GYRO_CONFIG_FCHOICE_B_8K;
        acc_cfg  = ACCEL_CONFIG_FS_SEL_8G;
        acc_cfg2 = ACCEL_CONFIG2_FCHOICE_B_4K | ACCEL_CONFIG2_DLPF_1046HZ_503US;

        this->gyro_factor_  = 2000.0f / 32768.0f;
        this->acc_factor_   = 8.0f / 32768.0f;
        this->read_term_us_ = 125;
    }
    else
    {
        config   = CONFIG_DLPF_184HZ_2900US;
        gyro_cfg = GYRO_CONFIG_FS_SEL_1000DPS | GYRO_CONFIG_FCHOICE_B_1K;
        acc_cfg  = ACCEL_CONFIG_FS_SEL_8G;
        acc_cfg2 = ACCEL_CONFIG2_FCHOICE_B_1K | ACCEL_CONFIG2_DLPF_218HZ_1880US;

        this->gyro_factor_  = 1000.0f / 32768.0f;
        this->acc_factor_   = 8.0f / 32768.0f;
        this->read_term_us_ = 1000;
    }

    write_reg(this, _Regs_PWR_MGMT_1, PWR_MGMT_1_H_RESET);

    do {

        sys_delay_millis(1);
    }
    while (read_reg(this, _Regs_PWR_MGMT_1) & PWR_MGMT_1_H_RESET);

    sys_delay_millis(3);

    do {

        if (read_reg(this, _Regs_WHO_AM_I) == MPU9255_WHOAMI)
            break;

        return;
    }
    while (false);

    write_reg(this, _Regs_USER_CTRL, USER_CTRL_I2C_IF_DIS | USER_CTRL_FIFO_RST | USER_CTRL_I2C_MST_RST | USER_CTRL_SIG_COND_RST);

    do {

        sys_delay_millis(1);
    }
    while (read_reg(this, _Regs_USER_CTRL) & (USER_CTRL_FIFO_RST | USER_CTRL_I2C_MST_RST | USER_CTRL_SIG_COND_RST));

    write_reg(this, _Regs_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO);
    sys_delay_millis(1);
    write_reg(this, _Regs_PWR_MGMT_2, PWR_MGMT_2_GYRO_ACCEL_EN);
    sys_delay_millis(1);
    write_reg(this, _Regs_SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_GYRO | SIGNAL_PATH_RESET_ACCEL | SIGNAL_PATH_RESET_TEMP);
    sys_delay_millis(10);
    write_reg(this, _Regs_SMPLRT_DIV, SMPLRT_DIV_1);
    sys_delay_millis(10);
    write_reg(this, _Regs_CONFIG, config);
    sys_delay_millis(1);
    write_reg(this, _Regs_GYRO_CONFIG, gyro_cfg);
    sys_delay_millis(1);
    write_reg(this, _Regs_ACCEL_CONFIG, acc_cfg);
    sys_delay_millis(1);
    write_reg(this, _Regs_ACCEL_CONFIG2, acc_cfg2);
    sys_delay_millis(1);
    write_reg(this, _Regs_INT_PIN_CFG, INT_PIN_CFG_LATCH_INT_EN | INT_PIN_CFG_ANYRD_2CLEAR);
    sys_delay_millis(1);
    write_reg(this, _Regs_INT_ENABLE, INT_ENABLE_RAW_DRY_EN);
}

MpuHandle drv_mpu_init(MpuDev dev, MpuMode mode)
{
    if (dev < _MpuDev_N)
    {
        mpu_data[dev].spi_ = soc_spi_init(_SpiPort_1, _GpioPin_3);
        soc_spi_set_speed(mpu_data[dev].spi_, MPU9255_SPI_SPEED_KHZ_INIT);
        soc_spi_unlock(mpu_data[dev].spi_);

        init_reg(&mpu_data[dev], mode);

        soc_spi_set_speed(mpu_data[dev].spi_, MPU9255_SPI_SPEED_KHZ_READ);

        return (MpuHandle)(&mpu_data[dev]);
    }

    return NULL;
}

bool drv_mpu_is_ready(MpuHandle h, uint32_t now_us)
{
    LocalData   *this = (LocalData*)h;

    if (this->status_ == _Status_Done)
    {
        return true;
    }
    else if ((this->status_ == _Status_Waiting) && ((now_us - this->last_read_us_) >= this->read_term_us_))
    {
        this->read_buffer_[0] = MPUREG_RD(_Regs_ACCEL_XOUT_H);

        soc_spi_lock(this->spi_);
        soc_spi_txrx(this->spi_, this->read_buffer_, this->read_buffer_, 1 + MPU9255_READ_LEN, _spi_callback, this);

        this->status_ = _Status_Reading;
        this->last_read_us_ = now_us;
    }

    return false;
}

void drv_mpu_read_data(MpuHandle h, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az)
{
    LocalData   *this = (LocalData*)h;

    /**
     * MPU9250 gyroscope/accelerometer sensor direction
     *      +y      z_up +
     *   -x    +x
     *      -y      z_dn -
     *
     * AHRS's NWU gyroscope/accelerometer sensor direction
     *      +x      z_up +
     *   +y    -y
     *      -x      z_dn -
     */

    *gy = ((int16_t)this->read_buffer_[ 9] << 8) | (int16_t)this->read_buffer_[10];
    *gx = ((int16_t)this->read_buffer_[11] << 8) | (int16_t)this->read_buffer_[12];
    *gz = ((int16_t)this->read_buffer_[13] << 8) | (int16_t)this->read_buffer_[14];

    *ay = ((int16_t)this->read_buffer_[1] << 8) | (int16_t)this->read_buffer_[2];
    *ax = ((int16_t)this->read_buffer_[3] << 8) | (int16_t)this->read_buffer_[4];
    *az = ((int16_t)this->read_buffer_[5] << 8) | (int16_t)this->read_buffer_[6];

    *ax = -*ax;
    *ay = -*ay;

    this->status_ = _Status_Waiting;
}

float drv_mpu_get_gyro_factor(MpuHandle h)
{
    LocalData   *this = (LocalData*)h;

    return this->gyro_factor_;
}

float drv_mpu_get_acc_factor(MpuHandle h)
{
    LocalData   *this = (LocalData*)h;

    return this->acc_factor_;
}

float drv_mpu_get_gyro_zero_tolerance(MpuHandle h)
{
    LocalData   *this = (LocalData*)h;

    return (float)MPU9255_ZERO_TOLERANCE_DPS * (1.0f / this->gyro_factor_);
}

/* end of file ****************************************************************************************************** */
