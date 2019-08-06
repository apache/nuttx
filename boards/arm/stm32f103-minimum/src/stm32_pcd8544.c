/****************************************************************************
 * config/stm32f103-minimum/src/stm32_pcd8544.c
 *
 *   Copyright (C) 2011, 2013, 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdio.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/pcd8544.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32f103_minimum.h"

#ifdef CONFIG_NX_LCDDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_SPI_PORTNO 1   /* On SPI1 */

#ifndef CONFIG_LCD_CONTRAST
# define CONFIG_LCD_CONTRAST 60
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct spi_dev_s *g_spidev;
FAR struct lcd_dev_s *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  /* Configure the GPIO pins */

  stm32_configgpio(STM32_LCD_RST);
  stm32_configgpio(STM32_LCD_CD);
  stm32_gpiowrite(STM32_LCD_RST, 1);
  stm32_gpiowrite(STM32_LCD_CD, 1);

  g_spidev = stm32_spibus_initialize(LCD_SPI_PORTNO);

  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", LCD_SPI_PORTNO);
      return -ENODEV;
    }

  stm32_gpiowrite(STM32_LCD_RST, 0);
  up_mdelay(10);
  stm32_gpiowrite(STM32_LCD_RST, 1);
  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  g_lcddev = pcd8544_initialize(g_spidev, lcddev);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SPI port 1 to LCD %d: %d\n", lcddev);
    }
  else
    {
      lcdinfo("SPI port 1 bound to LCD %d\n", lcddev);

      /* And turn the LCD on (CONFIG_LCD_MAXPOWER should be 1) */

      (void)g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);

      /* Set contrast to right value, otherwise background too dark */

      (void)g_lcddev->setcontrast(g_lcddev, CONFIG_LCD_CONTRAST);

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
