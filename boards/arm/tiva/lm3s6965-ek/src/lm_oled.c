/****************************************************************************
 * boards/arm/tiva/lm3s6965-ek/src/lm_oled.c
 *
 *   Copyright (C) 2010-2011, 2015 Gregory Nutt. All rights reserved.
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
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/p14201.h>

#include "tiva_gpio.h"
#include "tiva_ssi.h"

#include "lm3s6965-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define the CONFIG_LCD_RITDEBUG to enable detailed debug output (stuff you
 * would never want to see unless you are debugging this file).
 *
 * Verbose debug must also be enabled
 */

#ifndef CONFIG_DEBUG_INFO
#  undef CONFIG_LCD_RITDEBUG
#endif

#ifdef CONFIG_LCD_RITDEBUG
#  define riterr(format, ...) _info(format, ##__VA_ARGS__)
#  define oleddc_dumpgpio(m)  tiva_dumpgpio(OLEDDC_GPIO, m)
#  define oledcs_dumpgpio(m)  tiva_dumpgpio(OLEDCS_GPIO, m)
#else
#  define riterr(x...)
#  define oleddc_dumpgpio(m)
#  define oledcs_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_graphics_setup
 *
 * Description:
 *   Called by NX initialization logic to configure the OLED.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno)
{
  FAR struct spi_dev_s *spi;
  FAR struct lcd_dev_s *dev;

  /* Configure the OLED GPIOs */

  oledcs_dumpgpio("board_graphics_setup: After OLEDCS setup");
  oleddc_dumpgpio("board_graphics_setup: On entry");

  tiva_configgpio(OLEDDC_GPIO); /* PC7: OLED display data/control select (D/Cn) */
  tiva_configgpio(OLEDEN_GPIO); /* PC6: Enable +15V needed by OLED (EN+15V) */

  oleddc_dumpgpio("board_graphics_setup: After OLEDDC/EN setup");

  /* Get the SSI port (configure as a Freescale SPI port) */

  spi = tiva_ssibus_initialize(0);
  if (!spi)
    {
      gerr("ERROR: Failed to initialize SSI port 0\n");
    }
  else
    {
      /* Bind the SSI port to the OLED */

      dev = rit_initialize(spi, devno);
      if (!dev)
        {
          gerr("ERROR: Failed to bind SSI port 0 to OLED %d: %d\n", devno);
        }
     else
        {
          ginfo("Bound SSI port 0 to OLED %d\n", devno);

          /* And turn the OLED on (CONFIG_LCD_MAXPOWER should be 1) */

          dev->setpower(dev, CONFIG_LCD_MAXPOWER);
          return dev;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name:  tiva_ssicmddata
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
 ****************************************************************************/

int tiva_ssicmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Set GPIO to 1 for data, 0 for command */

      tiva_gpiowrite(OLEDDC_GPIO, !cmd);
      return OK;
    }

  return -ENODEV;
}
