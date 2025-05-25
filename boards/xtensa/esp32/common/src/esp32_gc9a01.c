/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_gc9a01.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <arch/board/board.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/config.h>
#include <nuttx/lcd/gc9a01.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/spi/spi.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp32_gpio.h"
#include "esp32_spi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcd = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  g_spidev = esp32_spibus_initialize(DISPLAY_SPI);
  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", DISPLAY_SPI);
      return -ENODEV;
    }

  lcdinfo("Initialized SPI port %d\n", DISPLAY_SPI);

  /* SPI RX is not used. Same pin is used as LCD Data/Command control */

  esp32_configgpio(DISPLAY_DC, OUTPUT);
  esp32_gpiowrite(DISPLAY_DC, true);

  /* Pull LCD_RESET high */

  esp32_configgpio(DISPLAY_RST, OUTPUT);
  esp32_gpiowrite(DISPLAY_RST, false);
  up_mdelay(50);
  esp32_gpiowrite(DISPLAY_RST, true);
  up_mdelay(50);

  /* Set full brightness */

  esp32_configgpio(DISPLAY_BCKL, OUTPUT);
  esp32_gpiowrite(DISPLAY_BCKL, true);
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

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  g_lcd = gc9a01_lcdinitialize(g_spidev);
  if (!g_lcd)
    {
      lcderr("ERROR: Failed to bind SPI port %d to LCD %d\n",
             DISPLAY_SPI, devno);
    }
  else
    {
      lcdinfo("SPI port %d bound to LCD %d\n", DISPLAY_SPI, devno);
      return g_lcd;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn the display off */

  g_lcd->setpower(g_lcd, 0);
}
