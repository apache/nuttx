/****************************************************************************
 * configs/lc823450-xgevk/src/lc823450_st7565.c
 *
 *   Copyright 2017 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7565.h>

#include "up_arch.h"
#include "lc823450_gpio.h"
#include "lc823450_spi.h"
#include "lc823450-xgevk.h"

#ifdef CONFIG_NX_LCDDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_SPI_PORTNO 0

#define GPIO_LCD_CS   (GPIO_PORT1 | GPIO_PINE | GPIO_MODE_OUTPUT | GPIO_VALUE_ONE)
#define GPIO_LCD_A0   (GPIO_PORT4 | GPIO_PIN9 | GPIO_MODE_OUTPUT | GPIO_VALUE_ONE)
#define GPIO_LCD_NRES (GPIO_PORT4 | GPIO_PIN8 | GPIO_MODE_OUTPUT | GPIO_VALUE_ONE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void lc823450_st7565_reset(FAR struct st7565_lcd_s *lcd, bool on);
static void lc823450_st7565_select(FAR struct st7565_lcd_s *lcd);
static void lc823450_st7565_deselect(FAR struct st7565_lcd_s *lcd);
static void lc823450_st7565_cmddata(FAR struct st7565_lcd_s *lcd, const uint8_t cmd);
static int lc823450_st7565_senddata(FAR struct st7565_lcd_s *lcd, FAR const uint8_t *data,
                                    int size);
static int lc823450_st7565_backlight(FAR struct st7565_lcd_s *lcd, int level);


static FAR struct spi_dev_s *g_spidev;
static FAR struct lcd_dev_s *g_lcddev;

static struct st7565_lcd_s g_st7565_dev =
{
  .reset     = lc823450_st7565_reset,
  .select    = lc823450_st7565_select,
  .deselect  = lc823450_st7565_deselect,
  .cmddata   = lc823450_st7565_cmddata,
  .senddata  = lc823450_st7565_senddata,
  .backlight = lc823450_st7565_backlight,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_st7565_reset
 ****************************************************************************/

static void lc823450_st7565_reset(FAR struct st7565_lcd_s *lcd, bool on)
{
  if (on)
    {
      lc823450_gpio_write(GPIO_LCD_NRES, false);
    }
  else
    {
      lc823450_gpio_write(GPIO_LCD_NRES, true);
    }
}

/****************************************************************************
 * Name: lc823450_st7565_select
 ****************************************************************************/

static void lc823450_st7565_select(FAR struct st7565_lcd_s *lcd)
{
  lc823450_gpio_write(GPIO_LCD_CS, false);
}

/****************************************************************************
 * Name: lc823450_st7565_deselect
 ****************************************************************************/

static void lc823450_st7565_deselect(FAR struct st7565_lcd_s *lcd)
{
  lc823450_gpio_write(GPIO_LCD_CS, true);
}

/****************************************************************************
 * Name: lc823450_st7565_cmddata
 ****************************************************************************/

static void lc823450_st7565_cmddata(FAR struct st7565_lcd_s *lcd, const uint8_t cmd)
{
  lc823450_gpio_write(GPIO_LCD_A0, !cmd);
}

/****************************************************************************
 * Name: lc823450_st7565_senddata
 ****************************************************************************/

static int lc823450_st7565_senddata(FAR struct st7565_lcd_s *lcd,
                                    FAR const uint8_t *data,
                                    int size)
{
  SPI_SNDBLOCK(g_spidev, data, size);
  return 0;
}

/****************************************************************************
 * Name: lc823450_st7565_backlight
 ****************************************************************************/

static int lc823450_st7565_backlight(FAR struct st7565_lcd_s *lcd, int level)
{
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  g_spidev = lc823450_spibus_initialize(LCD_SPI_PORTNO);
  DEBUGASSERT(NULL != g_spidev);

  g_spidev->ops->setmode(g_spidev, SPIDEV_MODE3);
  g_spidev->ops->setbits(g_spidev, 8);
  g_spidev->ops->setfrequency(g_spidev, 1000000);

  return 0;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  g_lcddev = st7565_initialize(&g_st7565_dev, lcddev);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SPI port 1 to LCD %d: %d\n", lcddev);
    }
  else
    {
      lcdinfo("SPI port 1 bound to LCD %d\n", lcddev);

      (void)g_lcddev->setcontrast(g_lcddev, CONFIG_LCD_MAXCONTRAST);

      /* And turn the LCD on (CONFIG_LCD_MAXPOWER should be 1) */

      (void)g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
      return g_lcddev;
    }

  return NULL;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
}

#endif /* CONFIG_NX_LCDDRIVER */
