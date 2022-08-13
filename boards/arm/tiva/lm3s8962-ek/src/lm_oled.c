/****************************************************************************
 * boards/arm/tiva/lm3s8962-ek/src/lm_oled.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/p14201.h>

#include "tiva_gpio.h"
#include "tiva_ssi.h"
#include "lm3s8962-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define the CONFIG_LCD_RITDEBUG to enable detailed debug output (stuff you
 * would never want to see unless you are debugging this file).
 *
 * Verbose debug must also be enabled
 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_INFO
#  undef CONFIG_DEBUG_GRAPHICS
#endif

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
 *   Called NX initialization logic to configure the OLED.
 *
 ****************************************************************************/

struct lcd_dev_s *board_graphics_setup(unsigned int devno)
{
  struct spi_dev_s *spi;
  struct lcd_dev_s *dev;

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
          gerr("ERROR: Failed to bind SSI port 0 to OLED %d\n", devno);
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

int tiva_ssicmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Set GPIO to 1 for data, 0 for command */

      tiva_gpiowrite(OLEDDC_GPIO, !cmd);
      return OK;
    }

  return -ENODEV;
}
