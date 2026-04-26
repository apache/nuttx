/****************************************************************************
 * include/nuttx/sensors/mt6816.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MT6816_H
#define __INCLUDE_NUTTX_SENSORS_MT6816_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/qencoder.h>

#if defined(CONFIG_SENSORS_MT6816)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 * Prerequisites:
 *
 * CONFIG_SPI
 *   Enables support for SPI drivers
 * CONFIG_SENSORS_MT6816
 *   Enables support for the MT6816 driver
 */

/* The device always operates in mode 3 */

#define MT6816_SPI_MODE            SPIDEV_MODE3 /* Mode 3 */

/* SPI frequency */

#define MT6816_SPI_MAXFREQUENCY    1000000      /* 1MHz */

/* Resolution ***************************************************************/

#define MT6816_MAX           0x3fff   /* Maximum value (14 bits) */

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
 * Name: mt6816_initialize
 *
 * Description:
 *   Initialize the MT6816 device.
 *
 * Input Parameters:
 *   spi  - An SPI driver instance.
 *
 * Returned Value:
 *   A new lower half encoder interface for the MT6816 on success;
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct qe_lowerhalf_s *mt6816_initialize(FAR struct spi_dev_s *spi,
                                             uint16_t devid);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_MT6816 */
#endif /* __INCLUDE_NUTTX_SENSORS_MT6816_H */
