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

#include "esp32_gpio.h"
#include "esp32_spi.h"

#ifdef CONFIG_LCD_SSD1680

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_SSD1680_GPIO_PIN_PWR) && (CONFIG_SSD1680_GPIO_PIN_PWR>=0)
static bool ssd1680_set_vcc(bool state)
{
  esp32_gpiowrite(CONFIG_SSD1680_GPIO_PIN_PWR, state);
  return true;
}
#endif

#if defined(CONFIG_SSD1680_GPIO_PIN_RST) && (CONFIG_SSD1680_GPIO_PIN_RST>=0)
static bool ssd1680_set_rst(bool state)
{
  esp32_gpiowrite(CONFIG_SSD1680_GPIO_PIN_RST, state);
  return true;
}
#endif

#if defined(CONFIG_SSD1680_GPIO_PIN_BUSY) && (CONFIG_SSD1680_GPIO_PIN_BUSY>=0)
static bool ssd1680_check_busy(void)
{
  return esp32_gpioread(CONFIG_SSD1680_GPIO_PIN_BUSY);
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
#else
  .set_vcc = NULL,
#endif
#if defined(CONFIG_SSD1680_GPIO_PIN_RST)  && (CONFIG_SSD1680_GPIO_PIN_RST >= 0)
  .set_rst = ssd1680_set_rst,
#else
  .set_rst = NULL,
#endif
#if defined(CONFIG_SSD1680_GPIO_PIN_BUSY) && (CONFIG_SSD1680_GPIO_PIN_BUSY >= 0)
  .check_busy = ssd1680_check_busy,
#else
  .check_busy = NULL,
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

#if defined(CONFIG_SSD1680_GPIO_PIN_DTA_CMD) && \
  (CONFIG_SSD1680_GPIO_PIN_DTA_CMD >= 0)
  esp32_configgpio(CONFIG_SSD1680_GPIO_PIN_DTA_CMD, OUTPUT);
#endif

#if defined(CONFIG_SSD1680_GPIO_PIN_PWR) && (CONFIG_SSD1680_GPIO_PIN_PWR >= 0)
  esp32_configgpio(CONFIG_SSD1680_GPIO_PIN_PWR, OUTPUT);
  lcdinfo("Using pin %d as PWR control\n", CONFIG_SSD1680_GPIO_PIN_PWR);
#else
  lcdinfo("PWR control line is disabled\n");
#endif
#if defined(CONFIG_SSD1680_GPIO_PIN_RST) && (CONFIG_SSD1680_GPIO_PIN_RST >= 0)
  esp32_configgpio(CONFIG_SSD1680_GPIO_PIN_RST, OUTPUT);
  lcdinfo("Using pin %d as RESET\n", CONFIG_SSD1680_GPIO_PIN_RST);
#elif
  lcdinfo("RESET line is disabled\n");
#endif

#if defined(CONFIG_SSD1680_GPIO_PIN_BUSY) && \
  (CONFIG_SSD1680_GPIO_PIN_BUSY >= 0)
  esp32_configgpio(CONFIG_SSD1680_GPIO_PIN_BUSY, INPUT | PULLUP);
  lcdinfo("Using pin %d for reading busy state\n",
          CONFIG_SSD1680_GPIO_PIN_BUSY);
#elif
  lcdinfo("Read busy line is disabled\n");
#endif

  /* Initialize SPI */

  spi = esp32_spibus_initialize(CONFIG_SSD1680_SPI_BUS);
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

#endif

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
