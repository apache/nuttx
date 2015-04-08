/********************************************************************************************
 * drivers/sensors/mpl115a.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 ********************************************************************************************/

#ifndef __DRIVERS_SENSORS_MPL115A_H
#define __DRIVERS_SENSORS_MPL115A_H

#if defined(CONFIG_MPL115A)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* Configuration ****************************************************************************/
/* Prerequisites:
 *
 * CONFIG_SENSORS_MPL115A
 *   Enables support for the MPL115A driver
 * CONFIG_MPL115A_REGDEBUG
 *   Enable very low register-level debug output.  Requires CONFIG_DEBUG.
 */

/* There are two types of MPL115A chips. The MPL115A1 communicates with the target CPU
 * via a SPI interface. The MPL115A2 communicates via I2C interface.
 * Note: This driver only supports MPL115A1 (SPI Interface).
 */

/* SPI **************************************************************************************/
/* The device always operates in mode 0 */

#define MPL115A_SPI_MODE            SPIDEV_MODE0 /* Mode 0 */

/* SPI frequency */

#define MPL115A_SPI_MAXFREQUENCY    800000       /* 8MHz */

/* MPL115A Registers ************************************************************************/
/* Register Addresses */

#define MPL115A_BASE_CMD            0x80
#define MPL115A_PADC_MSB            0x00  /* 10-bit Pressure ADC output value MSB */
#define MPL115A_PADC_LSB            0x01  /* 10-bit Pressure ADC output value LSB */
#define MPL115A_TADC_MSB            0x02  /* 10-bit Temperature ADC output value MSB */
#define MPL115A_TADC_LSB            0x03  /* 10-bit Temperature ADC output value LSB */
#define MPL115A_A0_MSB              0x04  /* a0 coefficient MSB */
#define MPL115A_A0_LSB              0x05  /* a0 coefficient LSB */
#define MPL115A_B1_MSB              0x06  /* b1 coefficient MSB */
#define MPL115A_B1_LSB              0x07  /* b1 coefficient LSB */
#define MPL115A_B2_MSB              0x08  /* b2 coefficient MSB */
#define MPL115A_B2_LSB              0x09  /* b2 coefficient LSB */
#define MPL115A_C12_MSB             0x0a  /* c12 coefficient MSB */
#define MPL115A_C12_LSB             0x0b  /* c12 coefficient LSB */
                                          /* 0x0c - 0x11 are reserved */
#define MPL115A_CONVERT             0x12  /* Start Pressure and Temperature Conversion */

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mpl115a_register
 *
 * Description:
 *   Register the MPL115A character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MPL115A
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mpl115a_register(FAR const char *devpath, FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_MPL115A */
#endif /* __DRIVERS_SENSORS_MPL115A_H */
