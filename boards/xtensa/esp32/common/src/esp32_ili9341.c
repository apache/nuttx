/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_ili9341.c
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
#include <inttypes.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/lcd/ili9341.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "esp32_gpio.h"
#include "esp32_spi.h"
#include "hardware/esp32_gpio_sigmap.h"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef DISPLAY_RST
#  error "DISPLAY_RST must be defined in board.h!"
#endif
#ifndef DISPLAY_DC
#  error "DISPLAY_DC must be defined in board.h!"
#endif
#ifndef DISPLAY_BCKL
#  error "DISPLAY_BCKL must be defined in board.h!"
#endif
#ifndef DISPLAY_SPI
#  error "DISPLAY_SPI must be defined in board.h!"
#endif

#ifdef CONFIG_ESP32_LCD_OVERCLOCK
#  define ILI9341_SPI_MAXFREQUENCY 40*1000*1000
#else
#  define ILI9341_SPI_MAXFREQUENCY 10*1000*1000
#endif

#ifndef CONFIG_SPI_CMDDATA
#  error "The ILI9341 driver requires CONFIG_SPI_CMDATA in the configuration"
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

struct ili93414ws_lcd_s
{
  struct ili9341_lcd_s dev;
  struct spi_dev_s *spi;
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

static void esp32_ili93414ws_select(struct ili9341_lcd_s *lcd);
static void esp32_ili93414ws_deselect(struct ili9341_lcd_s *lcd);
static int esp32_ili93414ws_backlight(struct ili9341_lcd_s *lcd,
                                      int level);
static int esp32_ili93414ws_sendcmd(struct ili9341_lcd_s *lcd,
                                    const uint8_t cmd);
static int esp32_ili93414ws_sendparam(struct ili9341_lcd_s *lcd,
                                      const uint8_t param);
static int esp32_ili93414ws_sendgram(struct ili9341_lcd_s *lcd,
                                     const uint16_t *wd, uint32_t nwords);
static int esp32_ili93414ws_recvparam(struct ili9341_lcd_s *lcd,
                                      uint8_t *param);
static int esp32_ili93414ws_recvgram(struct ili9341_lcd_s *lcd,
                                     uint16_t *wd, uint32_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ili93414ws_lcd_s g_lcddev;
static struct lcd_dev_s *g_lcd = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_ili93414ws_select
 *
 * Description:
 *   Select the SPI, lock and reconfigure if necessary
 *
 * Input Parameters:
 *   lcd  - Reference to the public driver structure
 *
 ****************************************************************************/

static void esp32_ili93414ws_select(struct ili9341_lcd_s *lcd)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  SPI_LOCK(priv->spi, true);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), true);
}

/****************************************************************************
 * Name: esp32_ili93414ws_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   lcd  - Reference to the public driver structure
 *
 ****************************************************************************/

static void esp32_ili93414ws_deselect(struct ili9341_lcd_s *lcd)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: esp32_ili93414ws_backlight
 *
 * Description:
 *   Set the backlight level of the connected display.
 *   NOTE: Currently this function either sets the brightness to the maximum
 *         level (level > 0) or turns the display off (level == 0). Although
 *         the ILI9341 chip provides an interface for configuring the
 *         backlight level via WRITE_DISPLAY_BRIGHTNESS (0x51), it depends on
 *         the actual circuit of the display device. Usually the backlight
 *         pins are hardwired to Vcc, making the backlight level setting
 *         effectless.
 *
 * Input Parameters:
 *   lcd   - Reference to the public driver structure
 *   level - Backlight level
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int esp32_ili93414ws_backlight(struct ili9341_lcd_s *lcd,
                                      int level)
{
  if (level > 0)
    {
      lcd->sendcmd(lcd, ILI9341_WRITE_CTRL_DISPLAY);
      lcd->sendparam(lcd, 0x24);
    }
  else
    {
      lcd->sendcmd(lcd, ILI9341_WRITE_CTRL_DISPLAY);
      lcd->sendparam(lcd, 0x0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_sendcmd
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

static int esp32_ili93414ws_sendcmd(struct ili9341_lcd_s *lcd,
                                    const uint8_t cmd)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  lcdinfo("%02x\n", cmd);

  SPI_SETBITS(priv->spi, 8);

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), true);
  SPI_SEND(priv->spi, cmd);
  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_sendparam
 *
 * Description:
 *   Send a parameter to the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - Parameter to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int esp32_ili93414ws_sendparam(struct ili9341_lcd_s *lcd,
                                      const uint8_t param)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  lcdinfo("param=%04x\n", param);

  SPI_SETBITS(priv->spi, 8);

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);
  SPI_SEND(priv->spi, param);

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_sendgram
 *
 * Description:
 *   Send a number of pixel words to the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   wd     - Reference to the words to send
 *   nwords - Number of words to send
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int esp32_ili93414ws_sendgram(struct ili9341_lcd_s *lcd,
                                     const uint16_t *wd, uint32_t nwords)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  lcdinfo("lcd:%p, wd=%p, nwords=%" PRIu32 "\n", lcd, wd, nwords);

  SPI_SETBITS(priv->spi, 16);
  SPI_SNDBLOCK(priv->spi, wd, nwords);

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_recvparam
 *
 * Description:
 *   Receive a parameter from the lcd driver.
 *
 * Input Parameters:
 *   lcd    - Reference to the ili9341_lcd_s driver structure
 *   param  - Reference to where parameter is received
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int esp32_ili93414ws_recvparam(struct ili9341_lcd_s *lcd,
                                      uint8_t *param)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  SPI_SETBITS(priv->spi, 8);

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY(0), false);

  *param = (uint8_t)(SPI_SEND(priv->spi, (uintptr_t)param) & 0xff);

  return OK;
}

/****************************************************************************
 * Name: esp32_ili93414ws_recvgram
 *
 * Description:
 *   Receive pixel words from the lcd driver gram.
 *
 * Input Parameters:
 *   lcd    - Reference to the public driver structure
 *   wd     - Reference to where the pixel words are received
 *   nwords - Number of pixel words to receive
 *
 * Returned Value:
 *   OK - On Success
 *
 ****************************************************************************/

static int esp32_ili93414ws_recvgram(struct ili9341_lcd_s *lcd,
                                     uint16_t *wd, uint32_t nwords)
{
  struct ili93414ws_lcd_s *priv = (struct ili93414ws_lcd_s *)lcd;

  lcdinfo("wd=%p, nwords=%" PRIu32 "\n", wd, nwords);

  SPI_SETBITS(priv->spi, 16);
  SPI_RECVBLOCK(priv->spi, wd, nwords);

  return OK;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware. The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with
 *   the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct ili93414ws_lcd_s *priv = &g_lcddev;
  struct spi_dev_s *spi;
  lcdinfo("Initializing LCD\n");

  if (g_lcd == NULL)
    {
      spi = esp32_spibus_initialize(DISPLAY_SPI);
      if (!spi)
        {
          lcderr("Failed to initialize SPI bus.\n");
          return -ENODEV;
        }

      priv->spi = spi;

      /* Initialize non-SPI GPIOs */

      esp32_configgpio(DISPLAY_DC, OUTPUT_FUNCTION_3);
      esp32_gpio_matrix_out(DISPLAY_DC, SIG_GPIO_OUT_IDX, 0, 0);

      esp32_configgpio(DISPLAY_RST, INPUT_FUNCTION_3);
      esp32_gpio_matrix_out(DISPLAY_RST, SIG_GPIO_OUT_IDX, 0, 0);

      esp32_configgpio(DISPLAY_BCKL, OUTPUT_FUNCTION_3);
      esp32_gpio_matrix_out(DISPLAY_BCKL, SIG_GPIO_OUT_IDX, 0, 0);

      /* Reset ILI9341 */

      up_mdelay(10);
      esp32_gpiowrite(DISPLAY_RST, false);
      up_mdelay(10);
      esp32_gpiowrite(DISPLAY_RST, true);
      up_mdelay(50);

      /* Configure SPI */

      SPI_SETMODE(priv->spi, SPIDEV_MODE0);
      SPI_SETBITS(priv->spi, 8);
      SPI_HWFEATURES(priv->spi, 0);
      SPI_SETFREQUENCY(priv->spi, ILI9341_SPI_MAXFREQUENCY);

      /* Initialize ILI9341 driver with necessary methods */

      priv->dev.select      = esp32_ili93414ws_select;
      priv->dev.deselect    = esp32_ili93414ws_deselect;
      priv->dev.sendcmd     = esp32_ili93414ws_sendcmd;
      priv->dev.sendparam   = esp32_ili93414ws_sendparam;
      priv->dev.recvparam   = esp32_ili93414ws_recvparam;
      priv->dev.sendgram    = esp32_ili93414ws_sendgram;
      priv->dev.recvgram    = esp32_ili93414ws_recvgram;
      priv->dev.backlight   = esp32_ili93414ws_backlight;

      g_lcd = ili9341_initialize(&priv->dev, 0);

      if (g_lcd != NULL)
        {
          /* Turn the LCD on at 100% power */

          g_lcd->setpower(g_lcd, CONFIG_LCD_MAXPOWER);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 *
 * Description:
 *   Return a reference to the LCD object for the specified LCD. This allows
 *   support for multiple LCD devices.
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

/****************************************************************************
 * Name: board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support.
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  lcdinfo("Terminating LCD\n");

  if (g_lcd != NULL)
    {
      /* Turn the display off */

      g_lcd->setpower(g_lcd, 0);

      g_lcd = NULL;
    }
}
