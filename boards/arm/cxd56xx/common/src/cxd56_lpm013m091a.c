/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_lpm013m091a.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/lpm013m091a.h>
#include <arch/board/board.h>

#include "cxd56_gpio.h"
#include "cxd56_spi.h"
#include "cxd56_pinconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef DISPLAY_RST
#  error "DISPLAY_RST must be defined in board.h !!"
#endif
#ifndef DISPLAY_DC
#  error "DISPLAY_DC must be defined in board.h !!"
#endif
#ifndef DISPLAY_SPI
#  error "DISPLAY_SPI must be defined in board.h !!"
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

#ifndef LPM013M091A_SPI_MAXFREQUENCY
#  define LPM013M091A_SPI_MAXFREQUENCY 20000000
#endif

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct lpm013m091a4ws_lcd_s
{
  struct lpm013m091a_lcd_s dev;
  struct spi_dev_s *spi;
};

static struct lpm013m091a4ws_lcd_s g_lcddev;
static struct lcd_dev_s *g_lcd = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_lpm013m091a4ws_select
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

static void cxd56_lpm013m091a4ws_select(struct lpm013m091a_lcd_s *lcd)
{
  struct lpm013m091a4ws_lcd_s *priv
    = (struct lpm013m091a4ws_lcd_s *)lcd;

  SPI_LOCK(priv->spi, true);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: cxd56_lpm013m091a4ws_deselect
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

static void cxd56_lpm013m091a4ws_deselect(struct lpm013m091a_lcd_s *lcd)
{
  struct lpm013m091a4ws_lcd_s *priv
    = (struct lpm013m091a4ws_lcd_s *)lcd;

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: cxd56_lpm013m091a4ws_backlight
 *
 * Description:
 *   Set the backlight level of the connected display.
 *
 * Input Parameters:
 *   spi   - Reference to the public driver structure
 *   level - backligth level
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int cxd56_lpm013m091a4ws_backlight(struct lpm013m091a_lcd_s *lcd,
                                          int level)
{
  if (level > 0)
    {
      lcd->sendcmd(lcd, 0x53);
      lcd->sendparam(lcd, 0x24);
    }
  else
    {
      lcd->sendcmd(lcd, 0x53);
      lcd->sendparam(lcd, 0x0);
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_lpm013m091a4ws_sendcmd
 *
 * Description:
 *   Send a command to the lcd driver.
 *
 * Input Parameters:
 *   lcd  - Reference to the lpm013m091a_lcd_s driver structure
 *   cmd  - command to send
 *
 * Returned Value:
 *   On success - OK
 *
 ****************************************************************************/

static int cxd56_lpm013m091a4ws_sendcmd(struct lpm013m091a_lcd_s *lcd,
                                        const uint8_t cmd)
{
  struct lpm013m091a4ws_lcd_s *priv
    = (struct lpm013m091a4ws_lcd_s *)lcd;

  lcdinfo("%02x\n", cmd);

  SPI_SETBITS(priv->spi, 8);

  /* FIXME: This function can be replaced with SPI_CMDDATA().
   * This feature is needed to enable CONFIG_SPI_CMDDATA and board specific
   * cmddata() function.
   */

  cxd56_gpio_write(DISPLAY_DC, false); /* Indicate CMD */
  SPI_SEND(priv->spi, cmd);
  cxd56_gpio_write(DISPLAY_DC, true);  /* Indicate DATA */

  return OK;
}

/****************************************************************************
 * Name: cxd56_lpm013m091a4ws_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the lpm013m091a_lcd_s driver structure
 *   param  - parameter to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int cxd56_lpm013m091a4ws_sendparam(struct lpm013m091a_lcd_s *lcd,
                                          const uint8_t param)
{
  struct lpm013m091a4ws_lcd_s *priv
    = (struct lpm013m091a4ws_lcd_s *)lcd;

  cxd56_gpio_write(DISPLAY_DC, true);  /* Indicate DATA */
  SPI_SEND(priv->spi, param);

  return OK;
}

/****************************************************************************
 * Name: cxd56_lpm013m091a4ws_sendgram
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the lpm013m091a_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - number of words to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int cxd56_lpm013m091a4ws_sendgram(struct lpm013m091a_lcd_s *lcd,
                                         const uint16_t *wd, uint32_t nwords)
{
  struct lpm013m091a4ws_lcd_s *priv
    = (struct lpm013m091a4ws_lcd_s *)lcd;

  lcdinfo("lcd:%p, wd=%p, nwords=%d\n", lcd, wd, nwords);

  SPI_SETBITS(priv->spi, 16);
  SPI_SNDBLOCK(priv->spi, wd, nwords);

  return OK;
}

/****************************************************************************
 * Name: cxd56_lpm013m091a4ws_recvparam
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the lpm013m091a_lcd_s driver structure
 *   param  - Reference to where parameter receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int cxd56_lpm013m091a4ws_recvparam(struct lpm013m091a_lcd_s *lcd,
                                          uint8_t *param)
{
  struct lpm013m091a4ws_lcd_s *priv
    = (struct lpm013m091a4ws_lcd_s *)lcd;

  cxd56_gpio_write(DISPLAY_DC, true);  /* Indicate DATA */
  *param = (uint8_t)(SPI_SEND(priv->spi, param) & 0xff);

  return OK;
}

/****************************************************************************
 * Name: cxd56_lpm013m091a4ws_recvgram
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

static int cxd56_lpm013m091a4ws_recvgram(struct lpm013m091a_lcd_s *lcd,
                                         uint16_t *wd, uint32_t nwords)
{
  lcdinfo("wd=%p, nwords=%d\n", wd, nwords);

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
  struct lpm013m091a4ws_lcd_s *priv = &g_lcddev;
  struct spi_dev_s *spi;
#if defined(CONFIG_CXD56_DMAC)
  DMA_HANDLE            hdl;
  dma_config_t          conf;
#endif

  lcdinfo("Initializing lcd\n");

  if (g_lcd == NULL)
    {
      spi = cxd56_spibus_initialize(DISPLAY_SPI);
      if (!spi)
        {
          lcderr("ERROR: Failed to initialize spi bus.\n");
          return -ENODEV;
        }

      priv->spi = spi;

#if defined(CONFIG_CXD56_DMAC)
      /* DMA settings */

      hdl = cxd56_dmachannel(DISPLAY_DMA_TXCH, DISPLAY_DMA_TX_MAXSIZE);
      if (hdl)
        {
          conf.channel_cfg = DISPLAY_DMA_TXCH_CFG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(DISPLAY_SPI, CXD56_SPI_DMAC_CHTYPE_TX,
                              hdl, &conf);
        }

      hdl = cxd56_dmachannel(DISPLAY_DMA_RXCH, DISPLAY_DMA_RX_MAXSIZE);
      if (hdl)
        {
          conf.channel_cfg = DISPLAY_DMA_RXCH_CFG;
          conf.dest_width  = CXD56_DMAC_WIDTH8;
          conf.src_width   = CXD56_DMAC_WIDTH8;
          cxd56_spi_dmaconfig(DISPLAY_SPI, CXD56_SPI_DMAC_CHTYPE_RX,
                              hdl, &conf);
        }
#endif

      /* Reset LPM013M091A */

      up_mdelay(10);
      cxd56_gpio_write(DISPLAY_RST, false);
      up_mdelay(10);
      cxd56_gpio_write(DISPLAY_RST, true);
      up_mdelay(50);

      /* Configure SPI */

      SPI_SETMODE(priv->spi, SPIDEV_MODE3);
      SPI_SETBITS(priv->spi, 8);
      SPI_HWFEATURES(priv->spi, 0);
      SPI_SETFREQUENCY(priv->spi, LPM013M091A_SPI_MAXFREQUENCY);

      /* Initialize LPM013M091A driver with necessary methods */

      priv->dev.select      = cxd56_lpm013m091a4ws_select;
      priv->dev.deselect    = cxd56_lpm013m091a4ws_deselect;
      priv->dev.sendcmd     = cxd56_lpm013m091a4ws_sendcmd;
      priv->dev.sendparam   = cxd56_lpm013m091a4ws_sendparam;
      priv->dev.recvparam   = cxd56_lpm013m091a4ws_recvparam;
      priv->dev.sendgram    = cxd56_lpm013m091a4ws_sendgram;
      priv->dev.recvgram    = cxd56_lpm013m091a4ws_recvgram;
      priv->dev.backlight   = cxd56_lpm013m091a4ws_backlight;

      g_lcd = lpm013m091a_initialize(&priv->dev, 0);
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

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev == 0)
    {
      return g_lcd;
    }

  return NULL;
}
