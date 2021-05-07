/****************************************************************************
 * include/nuttx/analog/ads1242.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#define ADS1242_REG_ACR_BIT_NDRDY       (1 << 7)
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
  ADS1242_X1     = 0,
  ADS1242_X2     = ADS1242_REG_SETUP_BIT_PGA0,
  ADS1242_X4     = ADS1242_REG_SETUP_BIT_PGA1,
  ADS1242_X8     = ADS1242_REG_SETUP_BIT_PGA1 | ADS1242_REG_SETUP_BIT_PGA0,
  ADS1242_X16    = ADS1242_REG_SETUP_BIT_PGA2,
  ADS1242_X32    = ADS1242_REG_SETUP_BIT_PGA2 | ADS1242_REG_SETUP_BIT_PGA0,
  ADS1242_X64    = ADS1242_REG_SETUP_BIT_PGA2 | ADS1242_REG_SETUP_BIT_PGA1,
  ADS1242_X128   = ADS1242_REG_SETUP_BIT_PGA2 | ADS1242_REG_SETUP_BIT_PGA1 |
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
 *   spi - An instance of the SPI interface to use to communicate with
 *         ADS1242
 *   osc_freq_hz - The frequency of the ADS1242 oscillator in Hz. Required
 *                 for calculating the minimum delay periods when accessing
 *                 the device via SPI.
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
