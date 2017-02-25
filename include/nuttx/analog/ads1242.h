/****************************************************************************
 * include/nuttx/analog/ads1242.h
 *
 *   Copyright (C) 2016, DS-Automotion GmbH. All rights reserved.
 *   Author: Alexander Entinger <a.entinger@ds-automotion.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_ANALOG_ADS1242_H
#define __INCLUDE_NUTTX_ANALOG_ADS1242_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_SPI) && defined(CONFIG_ADC_ADS1242)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/
/* Cmd: ANIOC_ADS2142_READ                    Arg: uint32_t *value
 * Cmd: ANIOC_ADS2142_SET_GAIN                Arg: uint8_t value
 * Cmd: ANIOC_ADS2142_SET_POSITIVE_INPUT      Arg: uint8_t value
 * Cmd: ANIOC_ADS2142_SET_NEGATIVE_INPUT      Arg: uint8_t value
 * Cmd: ANIOC_ADS2142_IS_DATA_READY           Arg: bool *value
 * Cmd: ANIOC_ADS2142_DO_SYSTEM_OFFSET_CALIB  Arg: None
 */

#define ANIOC_ADS2142_READ                    _ANIOC(AN_ADS2142_FIRST + 0)
#define ANIOC_ADS2142_SET_GAIN                _ANIOC(AN_ADS2142_FIRST + 1)
#define ANIOC_ADS2142_SET_POSITIVE_INPUT      _ANIOC(AN_ADS2142_FIRST + 2)
#define ANIOC_ADS2142_SET_NEGATIVE_INPUT      _ANIOC(AN_ADS2142_FIRST + 3)
#define ANIOC_ADS2142_IS_DATA_READY           _ANIOC(AN_ADS2142_FIRST + 4)
#define ANIOC_ADS2142_DO_SYSTEM_OFFSET_CALIB  _ANIOC(AN_ADS2142_FIRST + 5)

/* ADS1242 REGISTER *********************************************************/

#define ADS1242_REG_SETUP               (0x00)  /* Setup Register */
#define ADS1242_REG_MUX                 (0x01)  /* Multiplexer Control Register */
#define ADS1242_REG_ACR                 (0x02)  /* Analog Control Register */
#define ADS1242_REG_ODAC                (0x03)  /* Offset DAC */
#define ADS1242_REG_DIO                 (0x04)  /* Data I/O */
#define ADS1242_REG_DIR                 (0x05)  /* Direction Control for Data I/O */
#define ADS1242_REG_IOCON               (0x06)  /* I/O Configuration Register */

/* ADS1242 REGISTER Bit Definitions *****************************************/

/* SETUP */
#define ADS1242_REG_SETUP_BIT_BOCS      (1 << 3)
#define ADS1242_REG_SETUP_BIT_PGA2      (1 << 2)
#define ADS1242_REG_SETUP_BIT_PGA1      (1 << 1)
#define ADS1242_REG_SETUP_BIT_PGA0      (1 << 0)
/* MUX */
#define ADS1242_REG_MUX_BIT_PSEL3       (1 << 7)
#define ADS1242_REG_MUX_BIT_PSEL2       (1 << 6)
#define ADS1242_REG_MUX_BIT_PSEL1       (1 << 5)
#define ADS1242_REG_MUX_BIT_PSEL0       (1 << 4)
#define ADS1242_REG_MUX_BIT_NSEL3       (1 << 3)
#define ADS1242_REG_MUX_BIT_NSEL2       (1 << 2)
#define ADS1242_REG_MUX_BIT_NSEL1       (1 << 1)
#define ADS1242_REG_MUX_BIT_NSEL0       (1 << 0)
/* ACR */
#define ADS1242_REG_ACR_BIT_nDRDY       (1 << 7)
#define ADS1242_REG_ACR_BIT_UnB         (1 << 6)
#define ADS1242_REG_ACR_BIT_SPEED       (1 << 5)
#define ADS1242_REG_ACR_BIT_BUFEN       (1 << 4)
#define ADS1242_REG_ACR_BIT_BITORDER    (1 << 3)
#define ADS1242_REG_ACR_BIT_RANGE       (1 << 2)
#define ADS1242_REG_ACR_BIT_DR1         (1 << 1)
#define ADS1242_REG_ACR_BIT_DR0         (1 << 0)

/* ADS1242 SPI COMMANDS *****************************************************/

#define ADS1242_CMD_READ_DATA           (0x01)
#define ADS1242_CMD_READ_REGISTER       (0x10)
#define ADS1242_CMD_WRITE_REGISTER      (0x50)
#define ADS1242_CMD_SELF_OFFSET_CALIB   (0xF1)
#define ADS1242_CMD_SELF_GAIN_CALIB     (0xf2)
#define ADS1242_CMD_SYSTEM_OFFSET_CALIB (0xf3)
#define ADS1242_CMD_RESET               (0xfe)

/* SPI BUS PARAMETERS *******************************************************/

/* 100 kHz, SCLK period has to be at least 4 x tOsc period of ADS1242
 * oscillator circuit.
 */

#define ADS1242_SPI_FREQUENCY   (100000)

/* Device uses SPI Mode 1: CKPOL = 0, CKPHA = 1 */

#define ADS1242_SPI_MODE        (SPIDEV_MODE1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  ADS1242_x1     = 0,
  ADS1242_x2     = ADS1242_REG_SETUP_BIT_PGA0,
  ADS1242_x4     = ADS1242_REG_SETUP_BIT_PGA1,
  ADS1242_x8     = ADS1242_REG_SETUP_BIT_PGA1 | ADS1242_REG_SETUP_BIT_PGA0,
  ADS1242_x16    = ADS1242_REG_SETUP_BIT_PGA2,
  ADS1242_x32    = ADS1242_REG_SETUP_BIT_PGA2 | ADS1242_REG_SETUP_BIT_PGA0,
  ADS1242_x64    = ADS1242_REG_SETUP_BIT_PGA2 | ADS1242_REG_SETUP_BIT_PGA1,
  ADS1242_x128   = ADS1242_REG_SETUP_BIT_PGA2 | ADS1242_REG_SETUP_BIT_PGA1 |
                   ADS1242_REG_SETUP_BIT_PGA0
} ADS1242_GAIN_SELECTION;

typedef enum
{
  ADS1242_P_AIN0 = 0,
  ADS1242_P_AIN1 = ADS1242_REG_MUX_BIT_PSEL0,
  ADS1242_P_AIN2 = ADS1242_REG_MUX_BIT_PSEL1,
  ADS1242_P_AIN3 = ADS1242_REG_MUX_BIT_PSEL1 | ADS1242_REG_MUX_BIT_PSEL0,
  ADS1242_P_AIN4 = ADS1242_REG_MUX_BIT_PSEL2,
  ADS1242_P_AIN5 = ADS1242_REG_MUX_BIT_PSEL2 | ADS1242_REG_MUX_BIT_PSEL0,
  ADS1242_P_AIN6 = ADS1242_REG_MUX_BIT_PSEL2 | ADS1242_REG_MUX_BIT_PSEL1,
  ADS1242_P_AIN7 = ADS1242_REG_MUX_BIT_PSEL2 | ADS1242_REG_MUX_BIT_PSEL1 |
                   ADS1242_REG_MUX_BIT_PSEL0,
} ADS1242_POSITIVE_INPUT_SELECTION;

typedef enum
{
  ADS1242_N_AIN0 = 0,
  ADS1242_N_AIN1 = ADS1242_REG_MUX_BIT_NSEL0,
  ADS1242_N_AIN2 = ADS1242_REG_MUX_BIT_NSEL1,
  ADS1242_N_AIN3 = ADS1242_REG_MUX_BIT_NSEL1 | ADS1242_REG_MUX_BIT_NSEL0,
  ADS1242_N_AIN4 = ADS1242_REG_MUX_BIT_NSEL2,
  ADS1242_N_AIN5 = ADS1242_REG_MUX_BIT_NSEL2 | ADS1242_REG_MUX_BIT_NSEL0,
  ADS1242_N_AIN6 = ADS1242_REG_MUX_BIT_NSEL2 | ADS1242_REG_MUX_BIT_NSEL1,
  ADS1242_N_AIN7 = ADS1242_REG_MUX_BIT_NSEL2 | ADS1242_REG_MUX_BIT_NSEL1 |
                   ADS1242_REG_MUX_BIT_NSEL0,
} ADS1242_NEGATIVE_INPUT_SELECTION;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ads1242_register
 *
 * Description:
 *   Register the ADS1242 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/ads1242"
 *   spi - An instance of the SPI interface to use to communicate with ADS1242
 *   osc_freq_hz - The frequency of the ADS1242 oscillator in Hz. Required for
 *   calculating the minimum delay periods when accessing the device via SPI.
 *
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ads1242_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     uint32_t const osc_freq_hz);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_ADC_ADS1242 */
#endif /* __INCLUDE_NUTTX_ANALOG_ADS1242_H */
