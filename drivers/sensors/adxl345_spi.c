/****************************************************************************
 * drivers/sensors/adxl345_spi.c
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
