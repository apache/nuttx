/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/src/esp32c3_st7735.c
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
#include <nuttx/lcd/st7735.h>

#include "esp32c3_spi.h"
#include "esp32c3_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_SPI_PORTNO ESP32C3_SPI2
#define LCD_DC         CONFIG_ESP32C3_SPI2_MISOPIN
#define LCD_RST        CONFIG_ESP32C3_LCD_RSTPIN
#define LCD_BL         CONFIG_ESP32C3_LCD_BLPIN

#ifndef CONFIG_SPI_CMDDATA
#  error "The ST7735 driver requires CONFIG_SPI_CMDATA in the config"
#endif

#ifndef CONFIG_ESP32C3_SPI_SWCS
#  error "The ST7735 driver requires CONFIG_ESP32C3_SPI_SWCS in the config"
#endif

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
  g_spidev = esp32c3_spibus_initialize(LCD_SPI_PORTNO);
  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", LCD_SPI_PORTNO);
      return -ENODEV;
    }

  /* SPI RX is not used. Same pin is used as LCD Data/Command control */

  esp32c3_configgpio(LCD_DC, OUTPUT);
  esp32c3_gpiowrite(LCD_DC, true);

  /* Pull LCD_RESET high */

  esp32c3_configgpio(LCD_RST, OUTPUT);
  esp32c3_gpiowrite(LCD_RST, false);
  up_mdelay(1);
  esp32c3_gpiowrite(LCD_RST, true);
  up_mdelay(120);

  /* Set full brightness */

  esp32c3_configgpio(LCD_BL, OUTPUT);
  esp32c3_gpiowrite(LCD_BL, true);

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
  g_lcd = st7735_lcdinitialize(g_spidev);
  if (!g_lcd)
    {
      lcderr("ERROR: Failed to bind SPI port %d to LCD %d\n", LCD_SPI_PORTNO,
      devno);
    }
  else
    {
      lcdinfo("SPI port %d bound to LCD %d\n", LCD_SPI_PORTNO, devno);
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
