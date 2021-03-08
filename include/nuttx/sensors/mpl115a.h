/****************************************************************************
 * include/nuttx/sensors/mpl115a.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MPL115A_H
#define __INCLUDE_NUTTX_SENSORS_MPL115A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MPL115A)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *
 * CONFIG_SENSORS_MPL115A
 *   Enables support for the MPL115A driver
 * CONFIG_MPL115A_REGDEBUG
 *   Enable very low register-level debug output.
 *  Requires CONFIG_DEBUG_FEATURES.
 */

/* There are two types of MPL115A chips.
 * The MPL115A1 communicates with the target CPU via a SPI interface.
 * The MPL115A2 communicates via I2C interface.
 * Note: This driver only supports MPL115A1 (SPI Interface).
 */

/* SPI **********************************************************************/

/* The device always operates in mode 0 */

#define MPL115A_SPI_MODE            SPIDEV_MODE0 /* Mode 0 */

/* SPI frequency */

#define MPL115A_SPI_MAXFREQUENCY    800000       /* 8MHz */

/* MPL115A Registers ********************************************************/

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct spi_dev_s;

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

#endif /* CONFIG_SPI && CONFIG_SENSORS_MPL115A */
#endif /* __INCLUDE_NUTTX_SENSORS_MPL115A_H */
