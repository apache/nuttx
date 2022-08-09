/****************************************************************************
 * drivers/lcd/ssd1306_spi.c
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
#include <nuttx/lcd/ssd1306.h>
#include <nuttx/spi/spi.h>
#include "ssd1306.h"

#if defined(CONFIG_LCD_SSD1306) && defined(CONFIG_LCD_SSD1306_SPI)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssd1306_configspi
 *
 * Description:
 *
 ****************************************************************************/

void ssd1306_configspi(FAR struct spi_dev_s *spi)
{
  lcdinfo("Mode: %d Bits: 8 Frequency: %d\n",
          CONFIG_SSD1306_SPIMODE, CONFIG_SSD1306_FREQUENCY);

  /* Configure SPI for the SSD1306 */

  SPI_SETMODE(spi, CONFIG_SSD1306_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_SSD1306_FREQUENCY);
}

/****************************************************************************
 * Name: ssd1306_sendbyte
 *
 * Description:
 *   Write a value to an 8-bit value to SSD1306
 *
 ****************************************************************************/

int ssd1306_sendbyte(FAR struct ssd1306_dev_s *priv, uint8_t regval)
{
#ifdef CONFIG_LCD_SSD1306_REGDEBUG
  _err("->0x%02x\n", regval);
#endif

  /* Send byte value to display */

  SPI_SEND(priv->spi, regval);
  return OK;
}

/****************************************************************************
 * Name: ssd1306_sendblk
 *
 * Description:
 *   Write an array of bytes to SSD1306
 *
 ****************************************************************************/

int ssd1306_sendblk(FAR struct ssd1306_dev_s *priv,
                    uint8_t *data, uint8_t len)
{
  /* Send byte value to display */

  SPI_SNDBLOCK(priv->spi, data, len);
  return OK;
}

/****************************************************************************
 * Name: ssd1306_select
 *
 * Description:
 *   Enable/Disable SSD1306 SPI CS
 *
 ****************************************************************************/

void ssd1306_select(FAR struct ssd1306_dev_s *priv, bool cs)
{
  /* If we are selecting the device */

  if (cs == true)
    {
      /* If SPI bus is shared then lock and configure it */

      SPI_LOCK(priv->spi, true);
      ssd1306_configspi(priv->spi);
    }

  /* Select/deselect SPI device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(priv->devno), cs);

  /* If we are deselecting the device */

  if (cs == false)
    {
      /* Unlock bus */

      SPI_LOCK(priv->spi, false);
    }
}

/****************************************************************************
 * Name: ssd1306_cmddata
 *
 * Description:
 *   Select Command/Data mode for SSD1306
 *
 ****************************************************************************/

void ssd1306_cmddata(FAR struct ssd1306_dev_s *priv, bool cmd)
{
  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(priv->devno), cmd);
}
#endif /* CONFIG_LCD_SSD1306 && CONFIG_LCD_SSD1306_SPI */
