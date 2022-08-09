/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_ssd1351.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1351.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "stm32f4discovery.h"

#ifdef CONFIG_LCD_SSD1351

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The pin configurations here require that SPI1 is selected */

#ifndef CONFIG_STM32_SPI1
#  error "The OLED driver requires CONFIG_STM32_SPI1 in the configuration"
#endif

#ifndef CONFIG_SSD1351_SPI4WIRE
#  error "The configuration requires the SPI 4-wire interface"
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

  /* Configure the OLED GPIOs. This initial configuration is RESET low,
   * putting the OLED into reset state.
   */

  stm32_configgpio(GPIO_OLED_RESET);

  /* Wait a bit then release the OLED from the reset state */

  up_mdelay(20);
  stm32_gpiowrite(GPIO_OLED_RESET, true);

  /* Get the SPI1 port interface */

  spi = stm32_spibus_initialize(1);
  if (spi == NULL)
    {
      lcderr("ERROR: Failed to initialize SPI port 1\n");
    }
  else
    {
      /* Bind the SPI port to the OLED */

      dev = ssd1351_initialize(spi, devno);
      if (dev == NULL)
        {
          lcderr("ERROR: Failed to bind SPI port 1 to OLED %d\n", devno);
        }
     else
        {
          lcdinfo("Bound SPI port 1 to OLED %d\n", devno);

          /* And turn the OLED on */

          dev->setpower(dev, LCD_FULL_ON);
          return dev;
        }
    }

  return NULL;
}

#endif /* CONFIG_LCD_SSD1351 */
