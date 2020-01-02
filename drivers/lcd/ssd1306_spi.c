/****************************************************************************
 * drivers/lcd/ssd1306_spi.c
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
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

int ssd1306_sendblk(FAR struct ssd1306_dev_s *priv, uint8_t *data, uint8_t len)
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

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), cs);

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

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), cmd);
}
#endif /* CONFIG_LCD_SSD1306 && CONFIG_LCD_SSD1306_SPI */
