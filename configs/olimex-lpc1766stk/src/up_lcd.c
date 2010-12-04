/****************************************************************************
 * config/olimex-lpc1766stk/src/up_lcd.c
 * arch/arm/src/board/up_lcd.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * POSSPBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/nokia6100.h>

#include "lpc17_internal.h"
#include "lpc17stk_internal.h"

#ifdef defined(CONFIG_NX_LCDDRIVER) && defined(CONFIG_LCD_NOKIA6100) && defined(CONFIG_LPC17_SSP0)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Define the CONFIG_LCD_NOKIADBG to enable detailed debug output (stuff you
 * would never want to see unless you are debugging this file).
 *
 * Verbose debug must also be enabled
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_NOKIADBG
#endif

#ifdef CONFIG_LCD_NOKIADBG
#  define lcddbg(format, arg...)  vdbg(format, ##arg)
#  define lcd_dumpgpio(m) lm3s_dumpgpio(LPC1766STK_LCD_RST, m)
#else
#  define lcddbg(x...)
#  define lcd_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_nxdrvinit
 *
 * Description:
 *   Called NX initialization logic to configure the LCD.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *up_nxdrvinit(unsigned int devno)
{
  FAR struct spi_dev_s *spi;
  FAR struct lcd_dev_s *dev;

  /* Configure the LCD GPIOs */

  lcd_dumpgpio("up_nxdrvinit: On entry");
  lm3s_configgpio(LPC1766STK_LCD_RST);
  lm3s_configgpio(LPC1766STK_LCD_BL);
  lcd_dumpgpio("up_nxdrvinit: After GPIO setup");

  /* Reset the LCD */

  lpc17_gpiowrite(LPC1766STK_LCD_RST, false);
  up_usdelay(10);
  lpc17_gpiowrite(LPC1766STK_LCD_RST, true);
  up_msdelay(5);

  /* Get the SSP port (configure as a Freescale SPI port) */

  spi = up_spiinitialize(0);
  if (!spi)
    {
      glldbg("Failed to initialize SSP port 0\n");
    }
  else
    {
      /* Bind the SSP port to the LCD */

      dev = nokia_lcdinitialize(spi, devno);
      if (!dev)
        {
          glldbg("Failed to bind SSP port 0 to LCD %d: %d\n", devno);
        }
     else
        {
          gllvdbg("Bound SSP port 0 to LCD %d\n", devno);

          /* And turn the LCD on (CONFIG_LCD_MAXPOWER should be 1) */

          (void)dev->setpower(dev, CONFIG_LCD_MAXPOWER);
          return dev;
        }
    }
  return NULL;
}

#endif /* CONFIG_NX_LCDDRIVER && CONFIG_LCD_NOKIA6100 && CONFIG_LPC17_SSP0 */
