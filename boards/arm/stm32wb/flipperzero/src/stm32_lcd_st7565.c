/****************************************************************************
 * boards/arm/stm32wb/flipperzero/src/stm32_lcd_st7565.c
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

#include <stdio.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7565.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32wb_gpio.h"
#include "stm32wb_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LCD_CONTRAST
# define CONFIG_LCD_CONTRAST 0x5f
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void stm32wb_st7565_reset(struct st7565_lcd_s *lcd, bool on);
static void stm32wb_st7565_select(struct st7565_lcd_s *lcd);
static void stm32wb_st7565_deselect(struct st7565_lcd_s *lcd);
static void stm32wb_st7565_cmddata(struct st7565_lcd_s *lcd,
                                   const uint8_t cmd);
static int stm32wb_st7565_senddata(struct st7565_lcd_s *lcd,
                                   const uint8_t *data, int size);
static int stm32wb_st7565_backlight(struct st7565_lcd_s *lcd, int level);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcddev;

static struct st7565_lcd_s g_st7565_dev =
{
  .reset     = stm32wb_st7565_reset,
  .select    = stm32wb_st7565_select,
  .deselect  = stm32wb_st7565_deselect,
  .cmddata   = stm32wb_st7565_cmddata,
  .senddata  = stm32wb_st7565_senddata,
  .backlight = stm32wb_st7565_backlight
};

static void stm32wb_st7565_reset(struct st7565_lcd_s *lcd, bool on)
{
  stm32wb_gpiowrite(STM32WB_LCD_RST, !on);
}

static void stm32wb_st7565_select(struct st7565_lcd_s *lcd)
{
  stm32wb_gpiowrite(STM32WB_LCD_CS, 0);
}

static void stm32wb_st7565_deselect(struct st7565_lcd_s *lcd)
{
  stm32wb_gpiowrite(STM32WB_LCD_CS, 1);
}

static void stm32wb_st7565_cmddata(struct st7565_lcd_s *lcd,
                                   const uint8_t cmd)
{
  stm32wb_gpiowrite(STM32WB_LCD_A0, !cmd);
}

static int stm32wb_st7565_senddata(struct st7565_lcd_s *lcd,
                                   const uint8_t *data, int size)
{
  SPI_SNDBLOCK(g_spidev, data, size);
  return 0;
}

static int stm32wb_st7565_backlight(struct st7565_lcd_s *lcd, int level)
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
  stm32wb_configgpio(STM32WB_LCD_RST);
  stm32wb_configgpio(STM32WB_LCD_A0);

  g_spidev = stm32wb_spibus_initialize(STM32WB_LCD_SPINO);

  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", STM32WB_LCD_SPINO);
      return -ENODEV;
    }

  g_spidev->ops->setmode(g_spidev, SPIDEV_MODE3);
  g_spidev->ops->setbits(g_spidev, 8);
  g_spidev->ops->setfrequency(g_spidev, 1000000);

  stm32wb_gpiowrite(STM32WB_LCD_RST, 0);
  up_mdelay(1);
  stm32wb_gpiowrite(STM32WB_LCD_RST, 1);

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  g_lcddev = st7565_initialize(&g_st7565_dev, lcddev);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SPI port %d to LCD %d\n",
             STM32WB_LCD_SPINO, lcddev);
    }
  else
    {
      lcdinfo("SPI port %d bound to LCD %d\n",
              STM32WB_LCD_SPINO, lcddev);

      /* And turn the LCD on (CONFIG_LCD_MAXPOWER should be 1) */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);

      /* Set contrast to right value, otherwise background too dark */

      g_lcddev->setcontrast(g_lcddev, CONFIG_LCD_CONTRAST);

      return g_lcddev;
    }

  return NULL;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* TO-FIX */
}
