/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_ili9340.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
#include <nuttx/lcd/ili9340.h>
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

#ifndef ILI9340_SPI_MAXFREQUENCY
#  define ILI9340_SPI_MAXFREQUENCY 20000000
#endif

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct ili93404ws_lcd_s
{
  struct ili9340_lcd_s dev;
  struct spi_dev_s *spi;
};
static struct ili93404ws_lcd_s g_lcddev;
static struct lcd_dev_s *g_lcd = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_ili93404ws_select
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

static void cxd56_ili93404ws_select(FAR struct ili9340_lcd_s *lcd)
{
  FAR struct ili93404ws_lcd_s *priv = (FAR struct ili93404ws_lcd_s *)lcd;

  SPI_LOCK(priv->spi, true);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: cxd56_ili93404ws_deselect
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

static void cxd56_ili93404ws_deselect(FAR struct ili9340_lcd_s *lcd)
{
  FAR struct ili93404ws_lcd_s *priv = (FAR struct ili93404ws_lcd_s *)lcd;

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: cxd56_ili93404ws_backlight
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

static int cxd56_ili93404ws_backlight(FAR struct ili9340_lcd_s *lcd,
                                      int level)
{
  if (level > 0)
    {
      lcd->sendcmd(lcd, ILI9340_WRITE_CTRL_DISPLAY);
      lcd->sendparam(lcd, 0x24);
    }
  else
    {
      lcd->sendcmd(lcd, ILI9340_WRITE_CTRL_DISPLAY);
      lcd->sendparam(lcd, 0x0);
    }

  return OK;
}

/****************************************************************************
 * Name: cxd56_ili93404ws_sendcmd
 *
 * Description:
 *   Send a command to the lcd driver.
 *
 * Input Parameters:
 *   lcd  - Reference to the ili9340_lcd_s driver structure
 *   cmd  - command to send
 *
 * Returned Value:
 *   On success - OK
 *
 ****************************************************************************/

static int cxd56_ili93404ws_sendcmd(FAR struct ili9340_lcd_s *lcd,
                                    const uint8_t cmd)
{
  FAR struct ili93404ws_lcd_s *priv = (FAR struct ili93404ws_lcd_s *)lcd;

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
 * Name: cxd56_ili93404ws_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9340_lcd_s driver structure
 *   param  - parameter to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int cxd56_ili93404ws_sendparam(FAR struct ili9340_lcd_s *lcd,
                                      const uint8_t param)
{
  FAR struct ili93404ws_lcd_s *priv = (FAR struct ili93404ws_lcd_s *)lcd;

  cxd56_gpio_write(DISPLAY_DC, true);  /* Indicate DATA */
  SPI_SEND(priv->spi, param);

  return OK;
}

/****************************************************************************
 * Name: cxd56_ili93404ws_sendgram
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9340_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - number of words to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int cxd56_ili93404ws_sendgram(FAR struct ili9340_lcd_s *lcd,
                                     const uint16_t *wd, uint32_t nwords)
{
  FAR struct ili93404ws_lcd_s *priv = (FAR struct ili93404ws_lcd_s *)lcd;

  lcdinfo("lcd:%p, wd=%p, nwords=%d\n", lcd, wd, nwords);

  SPI_SETBITS(priv->spi, 16);
  SPI_SNDBLOCK(priv->spi, wd, nwords);

  return OK;
}

/****************************************************************************
 * Name: cxd56_ili93404ws_recvparam
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9340_lcd_s driver structure
 *   param  - Reference to where parameter receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int cxd56_ili93404ws_recvparam(FAR struct ili9340_lcd_s *lcd,
                                        uint8_t *param)
{
  FAR struct ili93404ws_lcd_s *priv = (FAR struct ili93404ws_lcd_s *)lcd;

  cxd56_gpio_write(DISPLAY_DC, true);  /* Indicate DATA */
  *param = (uint8_t)(SPI_SEND(priv->spi, param) & 0xff);

  return OK;
}

/****************************************************************************
 * Name: cxd56_ili93404ws_recvgram
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

static int cxd56_ili93404ws_recvgram(FAR struct ili9340_lcd_s *lcd,
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
  FAR struct ili93404ws_lcd_s *priv = &g_lcddev;
  FAR struct spi_dev_s *spi;
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

      /* Reset ILI9340 */

      up_mdelay(10);
      cxd56_gpio_write(DISPLAY_RST, false);
      up_mdelay(10);
      cxd56_gpio_write(DISPLAY_RST, true);
      up_mdelay(50);

      /* Configure SPI */

      SPI_SETMODE(priv->spi, SPIDEV_MODE3);
      SPI_SETBITS(priv->spi, 8);
      SPI_HWFEATURES(priv->spi, 0);
      SPI_SETFREQUENCY(priv->spi, ILI9340_SPI_MAXFREQUENCY);

      /* Initialize ILI9340 driver with necessary methods */

      priv->dev.select      = cxd56_ili93404ws_select;
      priv->dev.deselect    = cxd56_ili93404ws_deselect;
      priv->dev.sendcmd     = cxd56_ili93404ws_sendcmd;
      priv->dev.sendparam   = cxd56_ili93404ws_sendparam;
      priv->dev.recvparam   = cxd56_ili93404ws_recvparam;
      priv->dev.sendgram    = cxd56_ili93404ws_sendgram;
      priv->dev.recvgram    = cxd56_ili93404ws_recvgram;
      priv->dev.backlight   = cxd56_ili93404ws_backlight;

      g_lcd = ili9340_initialize(&priv->dev, 0);
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
