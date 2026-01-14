/****************************************************************************
 * drivers/analog/ads7046.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ads7046.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device uses SPI Mode 1: CKPOL = 0, CKPHA = 1 */

#define ADS7046_SPI_MODE  (SPIDEV_MODE1)
#define ADS7046_SPI_BITS  16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads7046_dev_s
{
  FAR struct spi_dev_s *spi;
  int devno;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI Helpers */

static void ads7046_lock(FAR struct spi_dev_s *spi);

static void ads7046_unlock(FAR struct spi_dev_s *spi);

/* Character driver methods */

static int ads7046_open(FAR struct file *filep);

static int ads7046_close(FAR struct file *filep);

static int ads7046_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations ads7046_fops =
{
  ads7046_open,     /* open */
  ads7046_close,    /* close */
  NULL,             /* read */
  NULL,             /* write */
  NULL,             /* seek */
  ads7046_ioctl,    /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ads7046_lock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, true);
}

static void ads7046_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

static void ads7046_config_spi(FAR struct spi_dev_s *spi)
{
  SPI_SETMODE(spi, ADS7046_SPI_MODE);
  SPI_SETBITS(spi, ADS7046_SPI_BITS);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ADS7046_FREQUENCY);
}

static int ads7046_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7046_dev_s *priv = inode->i_private;

  ads7046_config_spi(priv->spi);
  up_mdelay(100);

  return OK;
}

static int ads7046_close(FAR struct file *filep)
{
  UNUSED(filep);
  return OK;
}

/* As per the datasheet, the ADS7046 returns the following readings:
 * INPUT VOLTAGE (AINP - AINM)               DESCRIPTION                 HEX
 * --------------------------------------    ------------------------    ---
 * <= 1 LSB                                  Negative full-scale code    000
 * 1 LSB to 2 LSB                            -                           001
 * V_REF / 2 to V_REF / 2 + 1 LSB            Mid code                    7FF
 * V_REF / 2 + 1 LSB to V_REF / 2 + 2 LSB    -                           800
 * >= V_REF - 1 LSB                          Positive full-scale code    FFF
 */

static void ads7046_read_conversion_result(
  FAR const struct ads7046_dev_s *dev, FAR uint16_t *conversion_result,
  bool fast_unsafe)
{
  *conversion_result = 0;

  if (!fast_unsafe)
    {
      ads7046_lock(dev->spi);
      ads7046_config_spi(dev->spi);
    }

  SPI_SELECT(dev->spi, SPIDEV_ADC(dev->devno), true);

  SPI_RECVBLOCK(dev->spi, conversion_result, 1);

  SPI_SELECT(dev->spi, SPIDEV_ADC(dev->devno), false);
  if (!fast_unsafe)
    {
      ads7046_unlock(dev->spi);
    }

  *conversion_result = (*conversion_result & 0b0111111111111000) >> 3;
}

static void ads7046_offset_calibration(FAR const struct ads7046_dev_s *dev)
{
  ads7046_lock(dev->spi);
  ads7046_config_spi(dev->spi);
  SPI_SELECT(dev->spi, SPIDEV_ADC(dev->devno), true);

  SPI_SNDBLOCK(dev->spi, 0, 4);

  SPI_SELECT(dev->spi, SPIDEV_ADC(dev->devno), false);
  ads7046_unlock(dev->spi);

  ainfo("requested offset calibration\n");
}

static int ads7046_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ads7046_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* Read the result of an analog conversion */

      case ANIOC_ADS7046_READ:
        {
          FAR uint16_t *data = (FAR uint16_t *)((uintptr_t)arg);
          ads7046_read_conversion_result(priv, data, false);
          break;
        }

      /* Read the result of an analog conversion (skip SPI locking &
       * reconfiguring for faster transfers - HIC SUNT DRACONES!)
       */

      case ANIOC_ADS7046_READ_FASTUNSAFE:
        {
          FAR uint16_t *data = (FAR uint16_t *)((uintptr_t)arg);
          ads7046_read_conversion_result(priv, data, true);
          break;
        }

      /* Start an offset calibration */

      case ANIOC_ADS7046_OFFCAL:
        {
          ads7046_offset_calibration(priv);
          break;
        }

      /* Command was not recognized */

      default:
        {
          aerr("ERROR: Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7046_register
 *
 * Description:
 *   Register the ADS7046 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the driver to register. E.g., "/dev/adc0"
 *   spi      - An instance of the SPI interface to use.
 *   devno    - SPI device number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ads7046_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     unsigned int devno)
{
  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the ADS7046 device structure */

  struct ads7046_dev_s *priv = kmm_malloc(sizeof(struct ads7046_dev_s));
  if (priv == NULL)
    {
      aerr("ERROR: Failed to allocate instance %s (%d)\n", strerror(errno),
           errno);
      return -ENOMEM;
    }

  priv->spi = spi;
  priv->devno = devno;

  /* Register the character driver */

  int ret = register_driver(devpath, &ads7046_fops, 0666, priv);
  if (ret < 0)
    {
      aerr("ERROR: Failed to register driver: %s (%d)\n", strerror(-ret),
           ret);
      kmm_free(priv);
    }

  return ret;
}
