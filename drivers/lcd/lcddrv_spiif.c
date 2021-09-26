/****************************************************************************
 * drivers/lcd/lcddrv_spiif.c
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <nuttx/kmalloc.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcddrv_spiif.h>

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

struct lcddrv_spiif_lcd_s
{
  /* Publicly visible device structure */

  struct lcddrv_lcd_s dev;

  /* Reference to spi device structure */

  struct spi_dev_s *spi;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcddrv_spiif_backlight
 *
 * Description:
 *   Set the backlight level of the connected display.
 *
 * Input Parameters:
 *   spi   - Reference to the public driver structure
 *   level - backlight level
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int lcddrv_spiif_backlight(FAR struct lcddrv_lcd_s *lcd, int level)
{
  return spiif_backlight(lcd, level);
}

/****************************************************************************
 * Name: lcddrv_spiif_select
 *
 * Description:
 *   Select the SPI, locking and re-configuring if necessary
 *
 * Input Parameters:
 *   spi  - Reference to the public driver structure
 *   isCommand - Flag indicating is command mode
 *
 * Returned Value:
 *
 ****************************************************************************/

static void lcddrv_spiif_select(FAR struct lcddrv_lcd_s *lcd)
{
  FAR struct lcddrv_spiif_lcd_s *priv = (FAR struct lcddrv_spiif_lcd_s *)lcd;

  SPI_LOCK(priv->spi, true);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: lcddrv_spiif_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the public driver structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void lcddrv_spiif_deselect(FAR struct lcddrv_lcd_s *lcd)
{
  FAR struct lcddrv_spiif_lcd_s *priv = (FAR struct lcddrv_spiif_lcd_s *)lcd;

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: lcddrv_spiif_sendmulti
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the lcddrv_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - number of words to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int lcddrv_spiif_sendmulti(FAR struct lcddrv_lcd_s *lcd,
                                  FAR const uint16_t *wd, uint32_t nwords)
{
  FAR struct lcddrv_spiif_lcd_s *priv = (FAR struct lcddrv_spiif_lcd_s *)lcd;

  SPI_SETBITS(priv->spi, 16);
  for (uint32_t t = 0; t < nwords; t++)
    {
      SPI_SEND(priv->spi, *wd++);
    }

  SPI_SETBITS(priv->spi, 8);

  return OK;
};

/****************************************************************************
 * Name: lcddrv_spiif_recv
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the lcddrv_lcd_s driver structure
 *   param  - Reference to where parameter receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int lcddrv_spiif_recv(FAR struct lcddrv_lcd_s *lcd,
                             FAR uint8_t *param)
{
  FAR struct lcddrv_spiif_lcd_s *priv = (FAR struct lcddrv_spiif_lcd_s *)lcd;

  lcdinfo("param=%04x\n", param);
  SPI_RECVBLOCK(priv->spi, param, 1);
  return OK;
}

/****************************************************************************
 * Name: lcddrv_spiif_send
 *
 * Description:
 *   Send to the lcd
 *
 * Input Parameters:
 *   lcd    - Reference to the lcddrv_lcd_s driver structure
 *   param  - Reference to where parameter to send is located
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int lcddrv_spiif_send(FAR struct lcddrv_lcd_s *lcd,
                             const uint8_t param)
{
  uint8_t r;
  FAR struct lcddrv_spiif_lcd_s *priv = (FAR struct lcddrv_spiif_lcd_s *)lcd;

  r = SPI_SEND(priv->spi, param);
  return r;
}

/****************************************************************************
 * Name: lcddrv_spiif_sendcmd
 *
 * Description:
 *   Send command to the lcd
 *
 * Input Parameters:
 *   lcd    - Reference to the lcddrv_lcd_s driver structure
 *   param  - Reference to where parameter to send is located
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int lcddrv_spiif_sendcmd(FAR struct lcddrv_lcd_s *lcd,
                                const uint8_t param)
{
  uint8_t r;
  FAR struct lcddrv_spiif_lcd_s *priv = (FAR struct lcddrv_spiif_lcd_s *)lcd;

  lcdinfo("param=%04x\n", param);
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), true);
  r = SPI_SEND(priv->spi, param);
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);
  return r;
}

/****************************************************************************
 * Name: lcddrv_spiif_recvmulti
 *
 * Description:
 *   Receive pixel words from the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the public driver structure
 *   wd     - Reference to where the pixel words receive
 *   nwords - number of pixel words to receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int lcddrv_spiif_recvmulti(FAR struct lcddrv_lcd_s *lcd,
                                  FAR uint16_t *wd, uint32_t nwords)
{
  FAR struct lcddrv_spiif_lcd_s *priv = (FAR struct lcddrv_spiif_lcd_s *)lcd;

  lcdinfo("wd=%p, nwords=%d\n", wd, nwords);
  SPI_SETBITS(priv->spi, 16);
  SPI_RECVBLOCK(priv->spi, wd, nwords);
  SPI_SETBITS(priv->spi, 8);
  return OK;
}

/****************************************************************************
 * Name:  FAR struct lcddrv_lcd_s *lcddrv_spiif_initialize
 *
 * Description:
 *   Initialize the device structure to control the LCD Single chip driver.
 *
 * Input Parameters:
 *   spi : handle to the spi to use
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD control object
 *   for the specified LCDDRV LCD Single chip driver.
 *   NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct lcddrv_lcd_s *lcddrv_spiif_initialize(FAR struct spi_dev_s *spi)
{
  FAR struct lcddrv_spiif_lcd_s *priv = (FAR struct lcddrv_spiif_lcd_s *)
    kmm_zalloc(sizeof(struct lcddrv_spiif_lcd_s));

  if (!priv)
    {
      return NULL;
    }

  lcdinfo("initialize lcddrv spi subdriver\n");

  priv->spi = spi;

  if (!priv->spi)
    {
      kmm_free(priv);
      return 0;
    }

  SPI_SETFREQUENCY(spi, CONFIG_LCD_LCDDRV_SPEED);
  SPI_SETBITS(spi, 8);

  /* Hook in our driver routines */

  priv->dev.select      = lcddrv_spiif_select;
  priv->dev.deselect    = lcddrv_spiif_deselect;
  priv->dev.sendparam   = lcddrv_spiif_send;
  priv->dev.sendcmd     = lcddrv_spiif_sendcmd;
  priv->dev.recvparam   = lcddrv_spiif_recv;
  priv->dev.sendgram    = lcddrv_spiif_sendmulti;
  priv->dev.recvgram    = lcddrv_spiif_recvmulti;
  priv->dev.backlight   = lcddrv_spiif_backlight;

  return &priv->dev;
}
