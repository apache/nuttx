/****************************************************************************
 * configs/zkit-arm-1769/src/lpc17_lcd.c
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Manikandan <code@zilogic.com>
 *
 * Based on configs/lm3s6965-ek/src/up_oled.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include <nuttx/lcd/st7567.h>

#include "up_arch.h"
#include "up_internal.h"

#include "lpc17_gpio.h"
#include "lpc17_ssp.h"
#include "zkitarm_internal.h"

#ifdef CONFIG_NX_LCDDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG with
 * CONFIG_DEBUG_VERBOSE too)
 */

#undef LCD_DEBUG   /* Define to enable debug */
#undef LCD_VERBOSE /* Define to enable verbose debug */

#ifdef LCD_DEBUG
#  define leddbg  lldbg
#  ifdef LCD_VERBOSE
#    define ledvdbg lldbg
#  else
#    define ledvdbg(x...)
#  endif
#else
#  undef LCD_VERBOSE
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct spi_dev_s *spi;
FAR struct lcd_dev_s *dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  lpc17_configgpio(ZKITARM_OLED_RST);
  lpc17_configgpio(ZKITARM_OLED_RS);
  lpc17_gpiowrite(ZKITARM_OLED_RST, 1);
  lpc17_gpiowrite(ZKITARM_OLED_RS, 1);

  zkit_sspinitialize();
  spi = lpc17_sspinitialize(0);
  if (!spi)
    {
      glldbg("Failed to initialize SSP port 0\n");
      return 0;
    }

  lpc17_gpiowrite(ZKITARM_OLED_RST, 0);
  up_mdelay(1);
  lpc17_gpiowrite(ZKITARM_OLED_RST, 1);
  return 1;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  dev = st7567_initialize(spi, lcddev);
  if (!dev)
    {
      glldbg("Failed to bind SSI port 0 to OLCD %d: %d\n", lcddev);
    }
  else
    {
      gllvdbg("Bound SSI port 0 to OLCD %d\n", lcddev);

      /* And turn the OLCD on (CONFIG_LCD_MAXPOWER should be 1) */
      (void)dev->setpower(dev, CONFIG_LCD_MAXPOWER);
      return dev;
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

/******************************************************************************
 * Name:  lpc17_spicmddata
 *
 * Description:
 *   Set or clear the SD1329 D/Cn bit to select data (true) or command
 *   (false).  This function must be provided by platform-specific logic.
 *   This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

int lpc17_ssp0cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY)
    {
      /* Set GPIO to 1 for data, 0 for command */

      lpc17_gpiowrite(ZKITARM_OLED_RS, !cmd);
      return OK;
    }

  return -ENODEV;
}

#endif /* CONFIG_NX_LCDDRIVER */
