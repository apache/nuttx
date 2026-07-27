/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_st7735.c
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

/* ST7735S bring-up for the Waveshare RP2350-LCD-0.96 (RP2350A + integrated
 * 0.96" 160x80 IPS panel).  Modelled on the RP2040 sibling
 * boards/arm/rp2040/common/src/rp2040_st7735.c, retargeted to rp23xx/SPI1.
 *
 * Pin map (silicon-confirmed by the EN-5/E5.0 bare-metal bring-up):
 *
 *   SCK  = GP10   (SPI1 SCK  function pin)
 *   DIN  = GP11   (SPI1 TX / MOSI function pin)
 *   DC   = GP8    (plain GPIO; see the note on SPI1 RX below)
 *   CS   = GP9    (driven by rp23xx_spi1select() in common/src/rp23xx_spi.c)
 *   RST  = GP12   (plain GPIO, active low)
 *   BL   = GP25   (plain GPIO, active high)
 *
 * NOTE on DC / SPI1 RX.  The panel bus is write-only (no MISO), so the SPI1
 * RX pin is unused as a receive line.  The rp23xx common board logic follows
 * the RP2040 convention of re-purposing that pin as the LCD Data/Command
 * pad: common/src/rp23xx_spi.c's rp23xx_spi1cmddata() drives
 * CONFIG_RP23XX_SPI1_RX_GPIO.  CONFIG_RP23XX_SPI1_RX_GPIO must therefore be
 * left at its default of 8 so that D/C lands on GP8.
 * rp23xx_common_initialize() puts that pin into SPI function at boot; we
 * claim it back as a SIO output here (rp23xx_gpio_init() selects
 * RP23XX_GPIO_FUNC_SIO).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7735.h>

#include "rp23xx_spi.h"
#include "rp23xx_gpio.h"

#ifdef CONFIG_LCD_ST7735

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_SPI_PORTNO 1

/* D/C shares the (unused) SPI1 RX pad -- see the file header note. */

#define LCD_DC         CONFIG_RP23XX_SPI1_RX_GPIO

/* RST/BL are plain GPIO and are the only genuinely board-specific pins here;
 * a board may override them from its <arch/board/board.h>.  The defaults are
 * the Waveshare RP2350-LCD-0.96 wiring confirmed by the EN-5/E5.0 bring-up.
 */

#ifndef LCD_RST
#  define LCD_RST      12
#endif

#ifndef LCD_BL
#  define LCD_BL       25
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcd;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  Brings up SPI1, claims the
 *   D/C, RST and BL pads as GPIO outputs, pulses the panel reset and
 *   binds the ST7735 driver.
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  if (g_lcd != NULL)
    {
      return OK;
    }

  g_spidev = rp23xx_spibus_initialize(LCD_SPI_PORTNO);
  if (g_spidev == NULL)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", LCD_SPI_PORTNO);
      return -ENODEV;
    }

  /* SPI RX is not used.  The same pin is the LCD Data/Command control, so
   * take it back from the SPI peripheral and drive it as a SIO output.
   */

  rp23xx_gpio_init(LCD_DC);
  rp23xx_gpio_setdir(LCD_DC, true);
  rp23xx_gpio_put(LCD_DC, true);

  /* Hardware reset pulse: high, low >=10ms, high, then let the panel settle.
   * The ST7735 driver issues no SWRESET of its own, so this is the only
   * reset the panel gets.
   */

  rp23xx_gpio_init(LCD_RST);
  rp23xx_gpio_setdir(LCD_RST, true);
  rp23xx_gpio_put(LCD_RST, true);
  up_mdelay(10);
  rp23xx_gpio_put(LCD_RST, false);
  up_mdelay(10);
  rp23xx_gpio_put(LCD_RST, true);
  up_mdelay(120);

  /* Backlight on (active high) */

  rp23xx_gpio_init(LCD_BL);
  rp23xx_gpio_setdir(LCD_BL, true);
  rp23xx_gpio_put(LCD_BL, true);

  /* Bind the ST7735 driver to the SPI bus.  This performs SLPOUT, COLMOD,
   * MADCTL, INVON (CONFIG_LCD_ST7735_INVCOLOR) and DISPON, and clears the
   * panel.
   */

  g_lcd = st7735_lcdinitialize(g_spidev);
  if (g_lcd == NULL)
    {
      lcderr("ERROR: Failed to bind SPI port %d to the ST7735\n",
             LCD_SPI_PORTNO);
      return -ENODEV;
    }

  g_lcd->setpower(g_lcd, CONFIG_LCD_MAXPOWER);

  lcdinfo("SPI port %d bound to the ST7735\n", LCD_SPI_PORTNO);
  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a reference to the LCD object for the specified LCD.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  if (devno != 0)
    {
      return NULL;
    }

  return g_lcd;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support.
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  if (g_lcd != NULL)
    {
      g_lcd->setpower(g_lcd, 0);
    }

  /* Backlight off */

  rp23xx_gpio_put(LCD_BL, false);
}

#endif /* CONFIG_LCD_ST7735 */
