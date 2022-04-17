/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpcxpresso-lpc1768/src/lpc17_40_oled.c
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
#include <nuttx/lcd/ug-9664hswag01.h>

#include "lpc17_40_gpio.h"
#include "lpc17_40_ssp.h"
#include "lpcxpresso-lpc1768.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This module is only built if CONFIG_NX_LCDDRIVER is selected.  In this
 * case, it would be an error if SSP1 is not also enabled.
 */

#ifndef CONFIG_LPC17_40_SSP1
#  error "The OLED driver requires CONFIG_LPC17_40_SSP1 in the configuration"
#endif

#ifndef CONFIG_UG9664HSWAG01_POWER
#  error "This logic requires CONFIG_UG9664HSWAG01_POWER in the configuration"
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define oleddc_dumpgpio(m)  lpc17_40_dumpgpio(LPCXPRESSO_OLED_POWER, m)
#  define oledcs_dumpgpio(m)  lpc17_40_dumpgpio(LPCXPRESSO_OLED_CS, m)
#else
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

struct lcd_dev_s *board_graphics_setup(unsigned int devno)
{
  struct spi_dev_s *spi;
  struct lcd_dev_s *dev;

  /* Configure the OLED GPIOs. For the SPI interface, insert jumpers in J42,
   * J43, J45 pin1-2 and J46 pin 1-2.
   */

  oledcs_dumpgpio("board_graphics_setup: After OLED CS setup");
  oleddc_dumpgpio("board_graphics_setup: On entry");

  lpc17_40_configgpio(LPCXPRESSO_OLED_POWER); /* OLED 11V power */
  lpc17_40_configgpio(LPCXPRESSO_OLED_DC);    /* OLED Command/Data */

  oleddc_dumpgpio("board_graphics_setup: After OLED Power/DC setup");

  /* Get the SPI1 port (configure as a Freescale SPI port) */

  spi = lpc17_40_sspbus_initialize(1);
  if (!spi)
    {
      gerr("ERROR: Failed to initialize SPI port 1\n");
    }
  else
    {
      /* Bind the SPI port to the OLED */

      dev = ug_initialize(spi, devno);
      if (!dev)
        {
          gerr("ERROR: Failed to bind SPI port 1 to OLED %d\n", devno);
        }
     else
        {
          ginfo("Bound SPI port 1 to OLED %d\n", devno);

          /* And turn the OLED on (dim) */

          dev->setpower(dev, UG_POWER_DIM);
          return dev;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name:  lpc17_40_ssp1cmddata
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

int lpc17_40_ssp1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Set GPIO to 1 for data, 0 for command */

      lpc17_40_gpiowrite(LPCXPRESSO_OLED_DC, !cmd);
      return OK;
    }

  return -ENODEV;
}

/****************************************************************************
 * Name:  ug_power
 *
 * Description:
 *   If the hardware supports a controllable OLED a power supply, this
 *   interface should be provided.  It may be called by the driver to turn
 *   the OLED power on and off as needed.
 *
 * Input Parameters:
 *
 *   devno - A value in the range of 0 through
 *      CONFIG_UG9664HSWAG01_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *   on - true:turn power on, false: turn power off.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_UG9664HSWAG01_POWER
void ug_power(unsigned int devno, bool on)
{
  ginfo("power %s\n", on ? "ON" : "OFF");
  lpc17_40_gpiowrite(LPCXPRESSO_OLED_POWER, on);
}
#endif
