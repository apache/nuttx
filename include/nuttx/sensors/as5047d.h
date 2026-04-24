/****************************************************************************
 * include/nuttx/sensors/as5047d.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_SENSORS_AS5047D_H
#define __INCLUDE_NUTTX_SENSORS_AS5047D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/qencoder.h>

#if defined(CONFIG_SENSORS_AS5047D)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_SPI
 *   Enables support for SPI drivers
 * CONFIG_SENSORS_AS5047D
 *   Enables support for the AS5047D driver
 */

/* The device always operates in mode 1 */

#define AS5047D_SPI_MODE             SPIDEV_MODE1 /* Mode 1 */

/* SPI frequency */

#define AS5047D_SPI_MAXFREQUENCY     1000000      /* 1MHz */

/* IOCTL Commands ***********************************************************/

/* Arg: uint16_t* pointer */

#define QEIOC_AS5047D_DIAGNOSTICS    _QEIOC(QE_AS5047D_FIRST + 0)

/* Arg: uint16_t* pointer */

#define QEIOC_AS5047D_MAGNITUDE      _QEIOC(QE_AS5047D_FIRST + 1)

/* Resolution ***************************************************************/

#define AS5047D_RESOLUTION           16384        /* Resolution (14 bits) */
#define AS5047D_VALUE_MASK           0x3fff       /* Maximum value (14 bits) */

/* Register Definitions *****************************************************/

/* Register Addresses */

#define AS5047D_CMD_READ             0x4000       /* Flag indicating read attempt */
#define AS5047D_CMD_WRITE            0x0000       /* Flag indicating write attempt */
#define AS5047D_FLAG_ERR             0x4000       /* Error flag in the response */
#define AS5047D_REG_NOP              0x0000       /* NOP Register */
#define AS5047D_REG_ERRFL            0x0001       /* Error Register */
#define AS5047D_REG_PROG             0x0003       /* Programming Register */
#define AS5047D_REG_DIAAGC           0x3ffc       /* Diagnostics/AGC Register */
#define AS5047D_REG_MAG              0x3ffd       /* Magnitude Register */
#define AS5047D_REG_ANGLEUNC         0x3ffe       /* Uncompensated Angle Register */
#define AS5047D_REG_ANGLE            0x3fff       /* Angle Register */

/* Error Flags (ERRFL register bits) */

#define AS5047D_ERR_PARITY           (1 << 2)
#define AS5047D_ERR_INV_CMD          (1 << 1)
#define AS5047D_ERR_FRAMING          (1 << 0)

/* DIAAGC Register Bits */

#define AS5047D_DIAAGC_MAGL          (1 << 11)
#define AS5047D_DIAAGC_MAGH          (1 << 10)
#define AS5047D_DIAAGC_COF           (1 << 9)
#define AS5047D_DIAAGC_OCF           (1 << 8)

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
 * Name: as5047d_initialize
 *
 * Description:
 *   Initialize the AS5047D device.
 *
 * Input Parameters:
 *   spi       - An SPI driver instance.
 *   spi_devid - The SPI device ID used by the board specific logic.
 *
 * Returned Value:
 *   A new lower half encoder interface for the AS5047D on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *as5047d_initialize(FAR struct spi_dev_s *spi,
                                              int spi_devid);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_AS5047D */
#endif /* __INCLUDE_NUTTX_SENSORS_AS5047D_H */
