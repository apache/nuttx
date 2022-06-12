/****************************************************************************
 * include/nuttx/sensors/as5048a.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_AS5048A_H
#define __INCLUDE_NUTTX_SENSORS_AS5048A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/qencoder.h>

#if defined(CONFIG_SENSORS_AS5048A)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_SPI
 *   Enables support for SPI drivers
 * CONFIG_AS5048A
 *   Enables support for the AS5048A driver
 */

/* The device always operates in mode 0 */

#define AS5048A_SPI_MODE            SPIDEV_MODE1 /* Mode 1 */

/* SPI frequency */

#define AS5048A_SPI_MAXFREQUENCY    1000000      /* 1MHz */

/* IOCTL Commands ***********************************************************/

/* Arg: int32_t* pointer */

#define QEIOC_AS5048A_ZEROPOSITION    _QEIOC(QE_AS5048A_FIRST+0)

/* Arg: uint8_t* pointer */

#define QEIOC_AS5048A_AUTOGAINCTL     _QEIOC(QE_AS5048A_FIRST+1)

/* Arg: uint8_t* pointer */

#define QEIOC_AS5048A_DIAGNOSTICS     _QEIOC(QE_AS5048A_FIRST+2)

/* Arg: int32_t* pointer */

#define QEIOC_AS5048A_MAGNITUDE       _QEIOC(QE_AS5048A_FIRST+3)

/* Resolution ***************************************************************/

#define AS5048A_MAX           0x3fff   /* Maximum value (14 bits) */

/* Register Definitions *****************************************************/

/* Register Addresses */

#define AS5048A_CMD_READ      0x4000   /* flag indicating read attempt */
#define AS5048A_CMD_WRITE     0x0000   /* flag indicating write attempt */
#define AS5048A_CLRERR_REG    0x1      /* clear error register */
#define AS5048A_PROG_REG      0x03     /* Programming Control Register */
#define AS5048A_ZEROLO_REG    0x17     /* Zero Position Register Bits 0~5 */
#define AS5048A_ZEROHI_REG    0x16     /* Zero Position Register Bits 6~13 */
#define AS5048A_AGC_REG       0x3ffd   /* Automatic Gain Control Register */
#define AS5048A_DIAG_REG      0x3ffd   /* Diagnostics Register */
#define AS5048A_MAG_REG       0x3ffe   /* Magnitude Register Bits 0~13 */
#define AS5048A_ANGLE_REG     0x3fff   /* Angle Register Bits 0~13 */

/* Programming Control Register Bit Definitions */

#define AS5048A_PROG_ENABLE   (1 << 0)
#define AS5048A_PROG_BURN     (1 << 3)
#define AS5048A_PROG_VERIFY   (1 << 6)

/* Diagnostics Register Bit Definitions */

#define AS5048A_DIAG_OCF      (1 << 8)  /* Offset Compensation Finished */
#define AS5048A_DIAG_COF      (1 << 9)  /* Cordic Overflow */
#define AS5048A_DIAG_COMPLOW  (1 << 10) /* High Magnetic Field */
#define AS5048A_DIAG_COMPHIGH (1 << 11) /* Low Magnetic Field */

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Name: as5048a_initialize
 *
 * Description:
 *   Initialize the AS5048A device.
 *
 * Input Parameters:
 *   spi  - An SPI driver instance.
 *
 * Returned Value:
 *   A new lower half encoder interface for the AS5048A on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *as5048a_initialize(FAR struct spi_dev_s *spi,
                                              int spi_devid);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_AS5048A */
#endif /* __INCLUDE_NUTTX_SENSORS_AS5048A_H */
