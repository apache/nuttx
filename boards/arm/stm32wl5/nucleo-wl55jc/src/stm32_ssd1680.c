/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_ssd1680.c
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

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1680.h>
#include <nuttx/spi/spi.h>

#include "stm32wl5.h"
#include "nucleo-wl55jc.h"
#include "stm32wl5_gpio.h"
#include "stm32wl5_ssd1680.h"

#if defined(CONFIG_VIDEO_FB) && defined(CONFIG_LCD_FRAMEBUFFER)
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_LCD_SSD1680

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(GPIO_SSD1680_PWR)
static bool ssd1680_set_vcc(bool state)
{
  esp32_gpiowrite(GPIO_SSD1680_PWR, state);
  return true;
}
#endif

#if defined(CONFIG_SSD1680_GPIO_PIN_RST)
static bool ssd1680_set_rst(bool state)
{
  stm32wl5_gpiowrite(GPIO_SSD1680_RST, state);
  return true;
}
#endif

#if defined(CONFIG_SSD1680_GPIO_PIN_BUSY)
static bool ssd1680_check_busy(void)
{
  return stm32wl5_gpioread(GPIO_SSD1680_BUSY);
}
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s    *g_lcddev;
struct ssd1680_priv_s g_ssd1680_priv =
{
#if defined(CONFIG_SSD1680_GPIO_PIN_PWR) && (CONFIG_SSD1680_GPIO_PIN_PWR >= 0)
  .set_vcc = ssd1680_set_vcc,
#endif

#if defined(CONFIG_SSD1680_GPIO_PIN_RST)  && (CONFIG_SSD1680_GPIO_PIN_RST >= 0)
  .set_rst = ssd1680_set_rst,
#endif

#if defined(CONFIG_SSD1680_GPIO_PIN_BUSY) && (CONFIG_SSD1680_GPIO_PIN_BUSY >= 0)
  .check_busy = ssd1680_check_busy,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct spi_dev_s *spi;

  /* Initialize additional I/O for e-ink display */

#if defined(GPIO_SSD1680_PWR)
  stm32wl5_configgpio(GPIO_SSD1680_PWR);   /* SSD1680 pwr */
  lcdinfo("SSD1680 power line is available (0x%08x)\n", GPIO_SSD1680_PWR);
#else
  lcdinfo("PWR control line is disabled\n");
#endif
#if defined(GPIO_SSD1680_RST)
  stm32wl5_configgpio(GPIO_SSD1680_RST);   /* SSD1680 reset */
  lcdinfo("SSD1680 reset line is available (0x%08x)\n", GPIO_SSD1680_RST);
#elif
  lcdinfo("SSD1680 RESET line is disabled\n");
#endif

#if defined(GPIO_SSD1680_BUSY)
  stm32wl5_configgpio(GPIO_SSD1680_BUSY);  /* SSD1680 busy */
  lcdinfo("SSD1680 Line for reading busy state is available (0x%08x)\n",
          GPIO_SSD1680_BUSY);
#elif
  lcdinfo("SSD1680 Read busy line is disabled\n");
#endif

  /* Initialize SPI */

  spi = stm32wl5_spibus_initialize(CONFIG_SSD1680_SPI_BUS);
  if (!spi)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n",
        CONFIG_SSD1680_SPI_BUS);
      return -ENODEV;
    }
  else
    {
      lcdinfo("Using SPI bus %d. SPI is initialized\n",
        CONFIG_SSD1680_SPI_BUS);
    }

  /* Bind the SPI port to the E-PAPER display */

  g_lcddev = ssd1680_initialize(spi, &g_ssd1680_priv);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SPI port %d to E-paper display\n",
          CONFIG_SSD1680_SPI_BUS);
      return -ENODEV;
    }
  else
    {
      lcdinfo("Bound SPI port %d to E-PAPER\n", CONFIG_SSD1680_SPI_BUS);

      /* And turn the OLED on.
       * Must be because setpower(1) function invokes the chip configuration
       */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
    }

  return OK;
}

/****************************************************************************
 * Name: board_ssd1680_getdev
 *
 * Description:
 *   Get the SSD1680 device driver instance
 *
 * Returned Value:
 *   Pointer to the instance
 *
 ****************************************************************************/

struct lcd_dev_s *board_ssd1680_getdev(void)
{
  return g_lcddev;
}

void board_lcd_uninitialize(void)
{
}

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  return board_ssd1680_getdev();
}

#endif
