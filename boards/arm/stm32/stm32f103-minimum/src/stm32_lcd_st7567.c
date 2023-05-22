/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_lcd_st7567.c
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
#include <nuttx/lcd/st7567.h>

#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32f103_minimum.h"

#ifdef CONFIG_NX_LCDDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_SPI_PORTNO 1   /* On SPI1 */

#ifndef CONFIG_LCD_CONTRAST
#  define CONFIG_LCD_CONTRAST 0x1f
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct spi_dev_s *g_spidev;
struct lcd_dev_s *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  stm32_configgpio(STM32_LCD_RST);
  stm32_configgpio(STM32_LCD_RS);
  stm32_gpiowrite(STM32_LCD_RST, 1);
  stm32_gpiowrite(STM32_LCD_RS, 1);

  g_spidev = stm32_spibus_initialize(LCD_SPI_PORTNO);

  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", LCD_SPI_PORTNO);
      return -ENODEV;
    }

  stm32_gpiowrite(STM32_LCD_RST, 0);
  up_mdelay(1);
  stm32_gpiowrite(STM32_LCD_RST, 1);
  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  g_lcddev = st7567_initialize(g_spidev, lcddev);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SPI port 1 to LCD %d\n", lcddev);
    }
  else
    {
      lcdinfo("SPI port 1 bound to LCD %d\n", lcddev);

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

#endif /* CONFIG_NX_LCDDRIVER */
