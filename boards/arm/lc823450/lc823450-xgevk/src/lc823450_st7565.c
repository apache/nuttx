/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450_st7565.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7565.h>

#include "arm_internal.h"
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

static void lc823450_st7565_reset(struct st7565_lcd_s *lcd, bool on);
static void lc823450_st7565_select(struct st7565_lcd_s *lcd);
static void lc823450_st7565_deselect(struct st7565_lcd_s *lcd);
static void lc823450_st7565_cmddata(struct st7565_lcd_s *lcd,
                                    const uint8_t cmd);
static int lc823450_st7565_senddata(struct st7565_lcd_s *lcd,
                                    const uint8_t *data,
                                    int size);
static int lc823450_st7565_backlight(struct st7565_lcd_s *lcd,
                                     int level);

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcddev;

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

static void lc823450_st7565_reset(struct st7565_lcd_s *lcd, bool on)
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

static void lc823450_st7565_select(struct st7565_lcd_s *lcd)
{
  lc823450_gpio_write(GPIO_LCD_CS, false);
}

/****************************************************************************
 * Name: lc823450_st7565_deselect
 ****************************************************************************/

static void lc823450_st7565_deselect(struct st7565_lcd_s *lcd)
{
  lc823450_gpio_write(GPIO_LCD_CS, true);
}

/****************************************************************************
 * Name: lc823450_st7565_cmddata
 ****************************************************************************/

static void lc823450_st7565_cmddata(struct st7565_lcd_s *lcd,
                                    const uint8_t cmd)
{
  lc823450_gpio_write(GPIO_LCD_A0, !cmd);
}

/****************************************************************************
 * Name: lc823450_st7565_senddata
 ****************************************************************************/

static int lc823450_st7565_senddata(struct st7565_lcd_s *lcd,
                                    const uint8_t *data,
                                    int size)
{
  SPI_SNDBLOCK(g_spidev, data, size);
  return 0;
}

/****************************************************************************
 * Name: lc823450_st7565_backlight
 ****************************************************************************/

static int lc823450_st7565_backlight(struct st7565_lcd_s *lcd, int level)
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

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  g_lcddev = st7565_initialize(&g_st7565_dev, lcddev);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SPI port 1 to LCD %d\n", lcddev);
    }
  else
    {
      lcdinfo("SPI port 1 bound to LCD %d\n", lcddev);

      g_lcddev->setcontrast(g_lcddev, CONFIG_LCD_MAXCONTRAST);

      /* And turn the LCD on (CONFIG_LCD_MAXPOWER should be 1) */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
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
