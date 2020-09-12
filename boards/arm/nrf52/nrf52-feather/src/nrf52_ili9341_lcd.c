/****************************************************************************
 * boards/arm/nrf52/nrf52-feather/src/nrf52_ili9341_lcd.c
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

#include <stdint.h>
#include <stdbool.h>
#include <endian.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ili9341.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "nrf52_gpio.h"
#include "nrf52_spi.h"

#include "nrf52-feather.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GRAM_ENDIAN_SWAP_BUFFER_LEN 127    /* Gram swap space */

#if (GRAM_ENDIAN_SWAP_BUFFER_LEN > 127)
#  error "GRAM_ENDIAN_SWAP_BUFFER_LEN must fit in a SPI transfer 127 words!"
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

struct ili93414ws_lcd_s
{
  struct ili9341_lcd_s dev;
  FAR struct spi_dev_s *spi;
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ili93414ws_lcd_s g_lcddev;
static struct lcd_dev_s *g_lcd = NULL;
static uint8_t swapbuf[GRAM_ENDIAN_SWAP_BUFFER_LEN * 2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ili93414ws_backlight
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

static int nrf52_ili93414ws_backlight(FAR struct ili9341_lcd_s *lcd,
                                      int level)
{
  return OK;
}

/****************************************************************************
 * Name: nrf52_ili93414ws_select
 *
 * Description:
 *   Select the SPI, locking and re-configuring if necessary
 *
 * Input Parameters:
 *   spi  - Reference to the public driver structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void nrf52_ili93414ws_select(FAR struct ili9341_lcd_s *lcd)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  SPI_LOCK(priv->spi, true);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: nrf52_ili93414ws_deselect
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

static void nrf52_ili93414ws_deselect(FAR struct ili9341_lcd_s *lcd)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  SPI_LOCK(priv->spi, false);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
}

/****************************************************************************
 * Name: nrf52_ili93414ws_sndcmd
 *
 * Description:
 *   Send a command to the lcd driver.
 *
 * Input Parameters:
 *   lcd  - Reference to the ili9341_lcd_s driver structure
 *   cmd  - command to send
 *
 * Returned Value:
 *   On success - OK
 *
 ****************************************************************************/

static int nrf52_ili93414ws_sendcmd(
    FAR struct ili9341_lcd_s *lcd, const uint8_t cmd)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  lcdinfo("%02x\n", cmd);

  nrf52_gpio_write(ILI9341_DISPLAY_DC, false); /* Indicate CMD */
  SPI_SEND(priv->spi, cmd);

  return OK;
}

/****************************************************************************
 * Name: nrf52_ili93414ws_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - parameter to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int nrf52_ili93414ws_sendparam(FAR struct ili9341_lcd_s *lcd,
                                      const uint8_t param)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  nrf52_gpio_write(ILI9341_DISPLAY_DC, true);  /* Indicate DATA */
  SPI_SEND(priv->spi, param);

  return OK;
}

/****************************************************************************
 * Name: nrf52_ili93414ws_sendgram
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - number of words to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int nrf52_ili93414ws_sendgram(FAR struct ili9341_lcd_s *lcd,
                                     const uint16_t *wd, uint32_t nwords)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;
  const uint8_t *bd = (uint8_t *)wd;
  uint32_t nbytes = nwords * 2;  /* nrf52 does not support 16bit transfers */
  uint8_t tx_len;
  uint16_t swap_idx;

  nrf52_gpio_write(ILI9341_DISPLAY_DC, true);  /* Indicate DATA */

  while (nbytes > 0)
    {
      tx_len = sizeof(swapbuf);
      if (nbytes < sizeof(swapbuf))
        {
          tx_len = nbytes;
        }

      /* The gram buffer must be swapped to BE since the transfer is
       * two 8bit MSB instead of one 16bit MSB and NRF52 is LE
       */

      for (swap_idx = 0; swap_idx < tx_len; swap_idx += 2)
        {
          *(uint16_t *)(swapbuf + swap_idx) = \
            htobe16(*(uint16_t *)(bd + swap_idx));
        }

      SPI_SNDBLOCK(priv->spi, swapbuf, tx_len);
      nbytes -= tx_len;
      bd += tx_len;
    }

  return OK;
};

/****************************************************************************
 * Name: nrf52_ili93414ws_recvparam
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - Reference to where parameter receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int nrf52_ili93414ws_recvparam(FAR struct ili9341_lcd_s *lcd,
                                        uint8_t *param)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;

  nrf52_gpio_write(ILI9341_DISPLAY_DC, true);  /* Indicate DATA */
  *param = (uint8_t)(SPI_SEND(priv->spi, 0xff) & 0xff);

  return OK;
}

/****************************************************************************
 * Name: nrf52_ili93414ws_recvgram
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

static int nrf52_ili93414ws_recvgram(FAR struct ili9341_lcd_s *lcd,
                                     uint16_t *wd, uint32_t nwords)
{
  FAR struct ili93414ws_lcd_s *priv = (FAR struct ili93414ws_lcd_s *)lcd;
  uint32_t nbytes = nwords * 2;  /* nrf52 does not support 16bit transfers */
  uint8_t rx_len;
  uint16_t swap_idx;

  nrf52_gpio_write(ILI9341_DISPLAY_DC, true);  /* Indicate DATA */

  while (nbytes > 0)
    {
      rx_len = sizeof(swapbuf);
      if (nbytes < sizeof(swapbuf))
        {
          rx_len = nbytes;
        }

      SPI_RECVBLOCK(priv->spi, swapbuf, rx_len);

      /* The gram buffer must be swapped from BE  to LE since the transfer
       * is two 8bit MSB instead of one 16bit MSB and NRF52 is LE
       */

      for (swap_idx = 0; swap_idx < rx_len; swap_idx += 2)
        {
          *(wd + swap_idx) = le16toh(*(uint16_t *)(swapbuf + swap_idx));
        }

      nbytes -= rx_len;
      wd += rx_len / 2;
    }

  return OK;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use,
 *   but with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  FAR struct ili93414ws_lcd_s *priv = &g_lcddev;
  FAR struct spi_dev_s *spi;

  lcdinfo("Initializing lcd\n");

  if (g_lcd == NULL)
    {
      nrf52_gpio_config(ILI9341_DISPLAY_DC);
      spi = nrf52_spibus_initialize(ILI9341_DISPLAY_SPI_DEV);
      if (!spi)
        {
          lcderr("ERROR: Failed to initialize spi bus.\n");
          return -ENODEV;
        }

      priv->spi = spi;

      /* Configure SPI */

      SPI_SETMODE(priv->spi, SPIDEV_MODE3);
      SPI_SETBITS(priv->spi, 8);
      SPI_HWFEATURES(priv->spi, 0);
      SPI_SETFREQUENCY(priv->spi, 8000000);

      /* Initialize structure */

      priv->dev.select      = nrf52_ili93414ws_select;
      priv->dev.deselect    = nrf52_ili93414ws_deselect;
      priv->dev.sendcmd     = nrf52_ili93414ws_sendcmd;
      priv->dev.sendparam   = nrf52_ili93414ws_sendparam;
      priv->dev.recvparam   = nrf52_ili93414ws_recvparam;
      priv->dev.sendgram    = nrf52_ili93414ws_sendgram;
      priv->dev.recvgram    = nrf52_ili93414ws_recvgram;
      priv->dev.backlight   = nrf52_ili93414ws_backlight;
      priv->spi             = spi;

      g_lcd = ili9341_initialize(&priv->dev, 0);
      g_lcd->setpower(g_lcd, 127);
      ili9341_clear(g_lcd, 0x0000);
    }

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev == 0)
    {
      return g_lcd;
    }

  return NULL;
}

void board_lcd_uninitialize(void)
{
}
