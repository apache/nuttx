/****************************************************************************
 * drivers/sensors/adxl345_spi.c
 *
 *   Copyright (C) 2014 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *   Based on STME811 driver
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/adxl345.h>

#include "adxl345.h"

#if defined(CONFIG_SENSORS_ADXL345) && defined(CONFIG_ADXL345_SPI)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl345_configspi
 *
 * Description:
 *
 ****************************************************************************/

static inline void adxl345_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the ADXL345 */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, ADXL345_SPI_MAXFREQUENCY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl345_getreg8
 *
 * Description:
 *   Read from an 8-bit ADXL345 register
 *
 ****************************************************************************/

uint8_t adxl345_getreg8(FAR struct adxl345_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval;

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  adxl345_configspi(priv->spi);

  /* Select the ADXL345 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, regaddr);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the ADXL345 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#ifdef CONFIG_ADXL345_REGDEBUG
  _err("%02x->%02x\n", regaddr, regval);
#endif
  return regval;
}

/****************************************************************************
 * Name: adxl345_putreg8
 *
 * Description:
 *   Write a value to an 8-bit ADXL345 register
 *
 ****************************************************************************/

void adxl345_putreg8(FAR struct adxl345_dev_s *priv, uint8_t regaddr,
                     uint8_t regval)
{
#ifdef CONFIG_ADXL345_REGDEBUG
  _err("%02x<-%02x\n", regaddr, regval);
#endif

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  adxl345_configspi(priv->spi);

  /* Select the ADXL345 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the ADXL345 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: adxl345_getreg16
 *
 * Description:
 *   Read 16-bits of data from an ADXL345 register
 *
 ****************************************************************************/

uint16_t adxl345_getreg16(FAR struct adxl345_dev_s *priv, uint8_t regaddr)
{
  uint16_t regval;

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  adxl345_configspi(priv->spi);

  /* Select the ADXL345 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, regaddr);
  SPI_RECVBLOCK(priv->spi, &regval, 2);

  /* Deselect the ADXL345 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#ifdef CONFIG_ADXL345_REGDEBUG
  _err("%02x->%04x\n", regaddr, regval);
#endif

  return regval;
}

#endif /* CONFIG_SENSORS_ADXL345 && CONFIG_ADXL345_SPI*/
