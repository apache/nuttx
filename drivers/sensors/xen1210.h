/********************************************************************************************
 * drivers/sensors/xen1210.h
 *
 *   Copyright (C) 2014 Alan Carvalho de Assis
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

#ifndef __DRIVERS_SENSORS_XEN1210_H
#define __DRIVERS_SENSORS_XEN1210_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/xen1210.h>

#if defined(CONFIG_SENSORS_XEN1210)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Driver support ***************************************************************************/
/* This format is used to construct the /dev/mag[n] device driver path.  It is defined here
 * so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/mag%d"
#define DEV_NAMELEN  16

/* Driver flags */

#define XEN1210_STAT_INITIALIZED  1 /* Device has been initialized */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

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

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

/********************************************************************************************
 * Name: xen1210_getdata
 *
 * Description:
 *   Read 24-bit from XEN1210 buffer, read three times (3 sensors)
 *
 ********************************************************************************************/

void xen1210_getdata(FAR struct xen1210_dev_s *priv);

/********************************************************************************************
 * Name: xen1210_putdata
 *
 * Description:
 *   Write 24-bit to XEN1210 buffer, write three times (3 sensors)
 *
 ********************************************************************************************/

void xen1210_putdata(FAR struct xen1210_dev_s *priv, uint32_t regval);

#endif /* CONFIG_SENSORS_XEN1210 */
#endif /* __DRIVERS_SENSORS_XEN1210_H */
