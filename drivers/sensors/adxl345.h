/********************************************************************************************
 * drivers/sensors/adxl345.h
 *
 *   Copyright (C) 2014 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *   using ADXL345 driver as template reference
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __DRIVERS_SENSORS_ADXL345_H
#define __DRIVERS_SENSORS_ADXL345_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/adxl345.h>

#if defined(CONFIG_SENSORS_ADXL345)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#ifdef CONFIG_ADXL345_I2C
#  error "Only the ADXL345 SPI interface is supported by this driver"
#endif

/* Driver support ***************************************************************************/
/* This format is used to construct the /dev/accel[n] device driver path.  It defined here
 * so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/accel%d"
#define DEV_NAMELEN  16

/* Driver flags */

#define ADXL345_STAT_INITIALIZED  1 /* Device has been initialized */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/
/* This defines type of events */

enum adxl345_event
{
  DATA_READY = 0,                      /* New data available */
  SINGLE_TAP,                          /* A tap event detected */
  DOUBLE_TAP,                          /* A double tap event detected */
  ACTIVITY,                            /* Activity detected */
  INACTIVITY,                          /* Inactivity detected */
  FREE_FAL,                            /* Free fall event */
  WATERMARK,                           /* Number samples in FIFO is equal to sample bits */
  OVERRUN,                             /* New data replaced unread data */
};

/* This defines operating mode */

enum adxl345_mode
{
  BYPASS_MODE = 0,                     /* Bypass FIFO, then it remain empty */
  FIFO_MODE,                           /* Sampled data are put in FIFO, up to 32 samples */
  STREAM_MODE,                         /* Sampled data are put in FIFO, when full remove old samples */
  TRIGGER_MODE                         /* Similar to Stream Mode, but when Trigger event happen FIFO freeze */
};

/* This structure describes the results of one ADXL345 sample */

struct adxl345_sample_s
{
  uint16_t data_x;                     /* Measured X-axis acceleration */
  uint16_t data_y;                     /* Measured Y-axis acceleration */
  uint8_t  data_z;                     /* Measured Z-axis acceleration */
};

/* This structure represents the state of the ADXL345 driver */

struct adxl345_dev_s
{
  /* Common fields */

  FAR struct adxl345_config_s *config; /* Board configuration data */
  sem_t exclsem;                       /* Manages exclusive access to this structure */
#ifdef CONFIG_ADXL345_SPI
  FAR struct spi_dev_s *spi;           /* Saved SPI driver instance */
#else
  FAR struct i2c_master_s *i2c;        /* Saved I2C driver instance */
#endif

  uint8_t status;                      /* See ADXL345_STAT_* definitions */
  struct work_s work;                  /* Supports the interrupt handling "bottom half" */

#ifdef CONFIG_ADXL345_REFCNT
  uint8_t crefs;                       /* Number of times the device has been opened */
#endif
  uint8_t nwaiters;                    /* Number of threads waiting for ADXL345 data */

  uint16_t ofsx;                       /* Offset X value */
  uint16_t ofsy;                       /* Offset Y value */
  uint16_t ofsz;                       /* Offset Z value */
  sem_t waitsem;                       /* Used to wait for the availability of data */

  struct work_s timeout;               /* Supports timeout work */
  struct adxl345_sample_s sample;      /* Last sampled accelerometer data */
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

/********************************************************************************************
 * Name: adxl345_getreg8
 *
 * Description:
 *   Read from an 8-bit ADXL345 register
 *
 ********************************************************************************************/

uint8_t adxl345_getreg8(FAR struct adxl345_dev_s *priv, uint8_t regaddr);

/********************************************************************************************
 * Name: adxl345_putreg8
 *
 * Description:
 *   Write a value to an 8-bit ADXL345 register
 *
 ********************************************************************************************/

void adxl345_putreg8(FAR struct adxl345_dev_s *priv, uint8_t regaddr, uint8_t regval);

/********************************************************************************************
 * Name: adxl345_getreg16
 *
 * Description:
 *   Read 16-bits of data from an ADXL345 register
 *
 ********************************************************************************************/

uint16_t adxl345_getreg16(FAR struct adxl345_dev_s *priv, uint8_t regaddr);

/********************************************************************************************
 * Name: adxl345_accworker
 *
 * Description:
 *   Handle accelerometer interrupt events (this function actually executes in the context of
 *   the worker thread).
 *
 ********************************************************************************************/

void adxl345_accworker(FAR struct adxl345_dev_s *priv, uint8_t intsta) weak_function;

#endif /* CONFIG_SENSORS_ADXL345 */
#endif /* __DRIVERS_SENSORS_ADXL345_H */
