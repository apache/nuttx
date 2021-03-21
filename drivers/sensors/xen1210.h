/****************************************************************************
 * drivers/sensors/xen1210.h
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

#ifndef __DRIVERS_SENSORS_XEN1210_H
#define __DRIVERS_SENSORS_XEN1210_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/xen1210.h>

#if defined(CONFIG_SENSORS_XEN1210)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/mag[n] device driver path.
 * It is defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/mag%d"
#define DEV_NAMELEN  16

/* Driver flags */

#define XEN1210_STAT_INITIALIZED  1 /* Device has been initialized */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the results of one XEN1210 sample */

struct xen1210_sample_s
{
  uint32_t data_x;                     /* Measured X-axis magnetic field */
  uint32_t data_y;                     /* Measured Y-axis magnetic field */
  uint32_t data_z;                     /* Measured Z-axis magnetic filed */
};

/* This structure represents the state of the XEN1210 driver */

struct xen1210_dev_s
{
  /* Common fields */

  FAR struct xen1210_config_s *config; /* Board configuration data */
  sem_t exclsem;                       /* Manages exclusive access to this structure */
  FAR struct spi_dev_s *spi;           /* Saved SPI driver instance */

  uint8_t status;                      /* See XEN1210_STAT_* definitions */
  struct work_s work;                  /* Supports the interrupt handling "bottom half" */

  uint8_t nwaiters;                    /* Number of threads waiting for XEN1210 data */

  sem_t waitsem;                       /* Used to wait for the availability of data */

  struct work_s timeout;               /* Supports timeout work */
  struct xen1210_sample_s sample;      /* Last sampled accelerometer data */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xen1210_getdata
 *
 * Description:
 *   Read 24-bit from XEN1210 buffer, read three times (3 sensors)
 *
 ****************************************************************************/

void xen1210_getdata(FAR struct xen1210_dev_s *priv);

/****************************************************************************
 * Name: xen1210_putdata
 *
 * Description:
 *   Write 24-bit to XEN1210 buffer, write three times (3 sensors)
 *
 ****************************************************************************/

void xen1210_putdata(FAR struct xen1210_dev_s *priv, uint32_t regval);

#endif /* CONFIG_SENSORS_XEN1210 */
#endif /* __DRIVERS_SENSORS_XEN1210_H */
