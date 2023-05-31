/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/
#define ISMRX_REG_POWER_CFG 0x00
#define ISMRX_REG_CFG 0x01
#define ISMRX_REG_CTRL 0x02
#define ISMRX_REG_OSC_FREQ 0x03
#define ISMRX_REG_OFF_TIMER_MSB 0x04
#define ISMRX_REG_OFF_TIMER_LSB 0x05
#define ISMRX_REG_CPU_RECOVERY_TIME 0x06
#define ISMRX_REG_RF_SETTLE_TIMER_MSB 0x07
#define ISMRX_REG_RF_SETTLE_TIMER_LSB 0x08
#define ISMRX_REG_STATUS 0x09
#define ISMRX_REG_AGC_DWELL_TIMER 0x0A

/**
 * @brief ISM RX description setting.
 * @details Specified setting for description of ISM RX Click driver.
 */
#define ISMRX_MODULATION_NULL 0
#define ISMRX_MODULATION_ASK 1
#define ISMRX_MODULATION_FSK 2

#define ISMRX_RECEIVE_MODE_NULL 0
#define ISMRX_RECEIVE_MODE_RX 3
#define ISMRX_RECEIVE_MODE_DRX 4

typedef enum
{
    ISMRX_OK = 0,
    ISMRX_ERROR = -1,
    ISMRX_CONTEXT_MODULATION_ERROR = -2,
    ISMRX_CONTEXT_RECEIVE_MODE_ERROR = -3

} ismtx_return_value_t;
/*!
 * @file ismrx.c
 * @brief ISM RX Click Driver.
 */

/**
 * @brief Dummy data.
 * @details Definition of dummy data.
 */
#define DUMMY 0x00

#define WRITE_DATA_CMD 0x10
#define READ_DATA_CMD 0x20
#define MASTER_RESET_CMD 0x30

#define ENABLE_DISALBE_MASK_VAL 0x1
#define OFF_TIMER_PRESCALE_MASK_VAL 0x3

#define CFG_MATRIX_DATA 19

#define COMMUNICATION_DELAY 1

#include "ISRMX.h"

// #define RH_SPI_WRITE_MASK 0x10 change this in RHGenericSPI.h

// class
ISRMX::ISRMX(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI &spi) : RHSPIDriver(slaveSelectPin, spi)

{
    modulation = ISMRX_MODULATION_NULL;
    receive_mode = ISMRX_RECEIVE_MODE_NULL;
}

// init
bool ISRMX::init()
{
    if (!RHSPIDriver::init())
        return false;

    // Reset the ISM RX
    bool ok = reset();
    Serial.println("reset" + ok ? "true" : "false");

    // Set the default configuration
    default_cfg();

    // fsk_config();

    return ok;
}

bool ISRMX::change_modulation(uint8_t modulation)
{

    delay(20);

    if (modulation == ISMRX_MODULATION_ASK)
    {

        Serial.println("ASK");
        default_cfg();

        return true;
    }
    else if (modulation == ISMRX_MODULATION_FSK)
    {
        Serial.println("FSK");
        fsk_config();

        return true;
    }
    else
    {
        return false;
    }
    // TODO CHECK IF SENSOR IS OK
}

// reset
bool ISRMX::reset()
{

    Serial.println("reset");
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
    selectSlave();
    status = _spi.transfer(MASTER_RESET_CMD); // send master reset command
    _spi.transfer(0x0);
    delayMicroseconds(1);
    deselectSlave();
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    delay(100);
    return status;
}

// default_cfg
void ISRMX::default_cfg()
{
    // set modulation
    modulation = ISMRX_MODULATION_ASK;
    // set receive mode
    receive_mode = ISMRX_RECEIVE_MODE_RX;

    spiWrite(ISMRX_REG_POWER_CFG, 0xE6);

    spiWrite(ISMRX_REG_CFG, 0x40);

    spiWrite(ISMRX_REG_CTRL, 0x06);
    spiWrite(ISMRX_REG_OSC_FREQ, 0x84);
}

void ISRMX::sleep_config()
{
    // set modulation
    modulation = ISMRX_MODULATION_ASK;
    // set receive mode
    receive_mode = ISMRX_RECEIVE_MODE_RX;

    spiWrite(ISMRX_REG_POWER_CFG, 0xE7); // chenge sleep bit (0 -> 1) at position 0 (LSB)

    spiWrite(ISMRX_REG_CFG, 0x40);

    spiWrite(ISMRX_REG_CTRL, 0x06);
    spiWrite(ISMRX_REG_OSC_FREQ, 0x84);
}

void ISRMX::fsk_config()
{
    modulation = ISMRX_MODULATION_ASK;
    receive_mode = ISMRX_RECEIVE_MODE_RX;

    // ismrx_generic_write( ctx, ISMRX_REG_POWER_CFG, 0x78 );
    // ismrx_generic_write( ctx, ISMRX_REG_CFG, 0x60 );
    // ismrx_generic_write( ctx, ISMRX_REG_CTRL, 0x01 );
    // ismrx_generic_write( ctx, ISMRX_REG_OSC_FREQ, 0x84 );

    spiWrite(ISMRX_REG_POWER_CFG, 0x78);
    spiWrite(ISMRX_REG_CFG, 0x60);
    spiWrite(ISMRX_REG_CTRL, 0x01);
    spiWrite(ISMRX_REG_OSC_FREQ, 0x84);
}

// read register

uint8_t ISRMX::read(uint8_t reg)
{

    uint8_t val = 0;
    ATOMIC_BLOCK_START;
    _spi.beginTransaction();
    selectSlave();
    _spi.transfer(reg | 0x02); // read command
    val = _spi.transfer(0);    // The written value is ignored, reg value is read
    deselectSlave();
    _spi.endTransaction();
    ATOMIC_BLOCK_END;
    return val;
    return spiRead(reg);
}

bool ISRMX::available()
{
    return true;
}

bool ISRMX::recv(uint8_t *buf, uint8_t *len)
{
    return true;
}

bool ISRMX::send(const uint8_t *data, uint8_t len)
{
    return true;
}

// maxMessageLength
uint8_t ISRMX::maxMessageLength()
{
    return 50; // dont know what to put here
}

// // void ismrx_cfg_setup ( ismrx_cfg_t *cfg )
// // {
// //     cfg->sck  = HAL_PIN_NC;
// //     cfg->miso = HAL_PIN_NC;
// //     cfg->mosi = HAL_PIN_NC;
// //     cfg->cs   = HAL_PIN_NC;

// //     cfg->spi_speed   = 1000000;
// //     cfg->spi_mode    = SPI_MASTER_MODE_0;
// //     cfg->cs_polarity = SPI_MASTER_CHIP_SELECT_POLARITY_ACTIVE_LOW;
// // }

// // err_t ismrx_init ( ismrx_t *ctx, ismrx_cfg_t *cfg )
// // {
// //     spi_master_config_t spi_cfg;

// //     spi_master_configure_default( &spi_cfg );

// //     ctx->modulation = ISMRX_MODULATION_NULL;
// //     ctx->receive_mode = ISMRX_RECEIVE_MODE_NULL;

// //     spi_cfg.sck  = cfg->sck;
// //     spi_cfg.miso = cfg->miso;
// //     spi_cfg.mosi = cfg->mosi;

// //     ctx->chip_select = cfg->cs;

// //     if ( spi_master_open( &ctx->spi, &spi_cfg ) == SPI_MASTER_ERROR )
// //     {
// //         return SPI_MASTER_ERROR;
// //     }

// //     if ( spi_master_set_default_write_data( &ctx->spi, DUMMY ) == SPI_MASTER_ERROR )
// //     {
// //         return SPI_MASTER_ERROR;
// //     }

// //     if ( spi_master_set_mode( &ctx->spi, cfg->spi_mode ) == SPI_MASTER_ERROR )
// //     {
// //         return SPI_MASTER_ERROR;
// //     }

// //     if ( spi_master_set_speed( &ctx->spi, cfg->spi_speed ) == SPI_MASTER_ERROR )
// //     {
// //         return SPI_MASTER_ERROR;
// //     }

// //     spi_master_set_chip_select_polarity( cfg->cs_polarity );
// //     spi_master_deselect_device( ctx->chip_select );

// //     return SPI_MASTER_SUCCESS;
// // }

// err_t ismrx_task_init ( ismrx_t *ctx, ismrx_cfg_t *cfg )
// {
//     spi_master_close( &ctx->spi );

//     err_t error_flag = digital_out_init( &ctx->mosi, cfg->mosi );
//     error_flag |= digital_in_init( &ctx->miso, cfg->miso );
//     return error_flag;
// }

// err_t ismrx_default_cfg ( ismrx_t *ctx )
// {
//     if ( ISMRX_RECEIVE_MODE_DRX == ctx->receive_mode )
//     {
//         ismrx_generic_write( ctx, ISMRX_REG_POWER_CFG, 0x00 );
//         ismrx_generic_write( ctx, ISMRX_REG_OSC_FREQ, 132 );
//         ismrx_generic_write( ctx, ISMRX_REG_POWER_CFG, 0xF8 );
//         ismrx_generic_write( ctx, ISMRX_REG_CFG, 0x61 );
//         ismrx_generic_write( ctx, ISMRX_REG_CTRL, 0x0B );
//         ismrx_generic_write( ctx, ISMRX_REG_OFF_TIMER_LSB, 0x03 );
//         ismrx_generic_write( ctx, ISMRX_REG_OFF_TIMER_MSB, 0x00 );
//         ismrx_generic_write( ctx, ISMRX_REG_CPU_RECOVERY_TIME, 0x01 );
//         ismrx_generic_write( ctx, ISMRX_REG_RF_SETTLE_TIMER_LSB, 0x01 );
//         ismrx_generic_write( ctx, ISMRX_REG_RF_SETTLE_TIMER_MSB, 0x00 );
//     }
//     else if ( ISMRX_RECEIVE_MODE_RX == ctx->receive_mode )
//     {
//         if ( ISMRX_MODULATION_ASK == ctx->modulation )
//         {
//             ismrx_generic_write( ctx, ISMRX_REG_POWER_CFG, 0xE6 );
//             ismrx_generic_write( ctx, ISMRX_REG_CFG, 0x40 );
//             ismrx_generic_write( ctx, ISMRX_REG_CTRL, 0x06 );
//             ismrx_generic_write( ctx, ISMRX_REG_OSC_FREQ, 0x84 );
//         }
//         else if ( ISMRX_MODULATION_FSK == ctx->modulation )
//         {
//             ismrx_generic_write( ctx, ISMRX_REG_POWER_CFG, 0x78 );
//             ismrx_generic_write( ctx, ISMRX_REG_CFG, 0x60 );
//             ismrx_generic_write( ctx, ISMRX_REG_CTRL, 0x01 );
//             ismrx_generic_write( ctx, ISMRX_REG_OSC_FREQ, 0x84 );
//         }
//         else
//         {
//             return ISMRX_CONTEXT_MODULATION_ERROR;
//         }
//     }
//     else
//     {
//         return ISMRX_CONTEXT_RECEIVE_MODE_ERROR;
//     }

//     return ISMRX_OK;
// }

// err_t ismrx_generic_write ( ismrx_t *ctx, uint8_t reg, uint8_t data_in )
// {
//     uint8_t tx_buf[ 2 ];
//     uint8_t cnt;

//     tx_buf[ 0 ] = reg & 0x0F;
//     tx_buf[ 0 ] |= WRITE_DATA_CMD;
//     tx_buf[ 1 ] = data_in;

//     spi_master_select_device( ctx->chip_select );
//     err_t error_flag = spi_master_write( &ctx->spi, tx_buf, 2 );
//     Delay_1us(  );
//     spi_master_deselect_device( ctx->chip_select );

//     return error_flag;
// }

// err_t ismrx_generic_read ( ismrx_t *ctx, uint8_t reg, uint8_t *data_out )
// {
//     uint8_t temp[ 2 ] = { 0 };

//     temp[ 0 ] = reg & 0x0F;
//     temp[ 0 ] |= READ_DATA_CMD;

//     spi_master_select_device( ctx->chip_select );
//     Delay_1us(  );
//     err_t error_flag = spi_master_write( &ctx->spi, temp, 2 );
//     Delay_1us(  );
//     spi_master_deselect_device( ctx->chip_select );
//     Delay_1us(  );
//     spi_master_select_device( ctx->chip_select );
//     Delay_1us(  );
//     error_flag |= spi_master_read( &ctx->spi, data_out, 1 );
//     Delay_1us(  );
//     spi_master_deselect_device( ctx->chip_select );

//     return error_flag;
// }

// // err_t ismrx_master_reset ( ismrx_t *ctx )
// // {
// //     uint8_t write_data[ 2 ] = { 0x00 };

// //     write_data[ 0 ] = MASTER_RESET_CMD;

// //     spi_master_select_device( ctx->chip_select );
// //     err_t error_flag = spi_master_write( &ctx->spi, write_data, 2 );
// //     Delay_1us(  );
// //     spi_master_deselect_device( ctx->chip_select );

// //     Delay_1sec(  );
// //     return error_flag;
// // }

// void ismrx_start_drx ( ismrx_t *ctx )
// {
//     digital_out_high( &ctx->mosi );
//     Delay_10ms(  );
//     digital_out_low( &ctx->mosi );
// }

// void ismrx_stop_drx ( ismrx_t *ctx )
// {
//     spi_master_select_device( ctx->chip_select );
//     spi_master_deselect_device( ctx->chip_select );
// }

// uint8_t ismrx_get_data ( ismrx_t *ctx )
// {
//     return digital_in_read( &ctx->miso );
// }

// // ------------------------------------------------------------------------- END