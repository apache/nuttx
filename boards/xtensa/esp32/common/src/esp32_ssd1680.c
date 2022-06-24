/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_ssd1680.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1680.h>
#include <nuttx/spi/spi.h>

#if defined(CONFIG_VIDEO_FB) && defined(CONFIG_LCD_FRAMEBUFFER)
#  include <nuttx/video/fb.h>
#endif

#include <arch/board/board.h>

#include "esp32_gpio.h"
#include "esp32_spi.h"

#ifdef CONFIG_LCD_SSD1680

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

#ifdef DISPLAY_VCC
static bool ssd1680_set_vcc(bool state);
#endif

#ifdef DISPLAY_RST
static bool ssd1680_set_rst(bool state);
#endif

#ifdef DISPLAY_BUSY
static bool ssd1680_check_busy(void);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s    *g_lcddev;

static struct ssd1680_priv_s g_ssd1680_priv =
{
#ifdef DISPLAY_VCC
  .set_vcc = ssd1680_set_vcc,
#endif
#ifdef DISPLAY_RST
  .set_rst = ssd1680_set_rst,
#endif
#ifdef DISPLAY_BUSY
  .check_busy = ssd1680_check_busy,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef DISPLAY_VCC
static bool ssd1680_set_vcc(bool state)
{
  esp32_gpiowrite(DISPLAY_VCC, state);
  return true;
}
#endif

#ifdef DISPLAY_RST
static bool ssd1680_set_rst(bool state)
{
  esp32_gpiowrite(DISPLAY_RST, state);
  return true;
}
#endif

#ifdef DISPLAY_BUSY
static bool ssd1680_check_busy(void)
{
  return esp32_gpioread(DISPLAY_BUSY);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  return g_lcddev;
}

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  int ret = ERROR;

  struct spi_dev_s *spi;

  /* Initialize additional I/O for e-ink display */

  esp32_configgpio(DISPLAY_DC, OUTPUT);

#ifdef DISPLAY_VCC
  esp32_configgpio(DISPLAY_VCC, OUTPUT);
  lcdinfo("Using pin %d as VCC control\n", DISPLAY_VCC);
#else
  lcdinfo("VCC line is disabled\n");
#endif

#ifdef DISPLAY_RST
  esp32_configgpio(DISPLAY_RST, OUTPUT);
  lcdinfo("Using pin %d as RESET\n", DISPLAY_RST);
#else
  lcdinfo("RESET line is disabled\n");
#endif

#ifdef DISPLAY_BUSY
  esp32_configgpio(DISPLAY_BUSY, INPUT | PULLUP);
  lcdinfo("Using pin %d for reading busy state\n",
          DISPLAY_BUSY);
#else
  lcdinfo("Read busy line is disabled\n");
#endif

  /* Initialize SPI */

  spi = esp32_spibus_initialize(DISPLAY_SPI_BUS);
  if (!spi)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", DISPLAY_SPI_BUS);
      return -ENODEV;
    }
  else
    {
      lcdinfo("Using SPI bus %d. SPI is initialized\n", DISPLAY_SPI_BUS);
    }

  /* Bind the SPI port to the E-PAPER display */

  g_lcddev = ssd1680_initialize(spi, &g_ssd1680_priv);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SPI port %d to E-paper display\n",
             DISPLAY_SPI_BUS);
      return -ENODEV;
    }
  else
    {
      lcdinfo("Bound SPI port %d to E-PAPER\n", DISPLAY_SPI_BUS);

      /* And turn the E-PAPER display on in order to clear.
       * Must be because setpower(1) function invokes the chip configuration
       */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
    }

  return ret;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* TO-FIX */
}

#endif
