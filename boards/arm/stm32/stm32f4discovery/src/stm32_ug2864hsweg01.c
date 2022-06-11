/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_ug2864hsweg01.c
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
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>

#include "stm32_gpio.h"
#include "stm32_spi.h"

#include "stm32f4discovery.h"

#ifdef CONFIG_LCD_UG2864HSWEG01

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* The pin configurations here require that SPI1 is selected */

#ifndef CONFIG_STM32_SPI1
#  error "The OLED driver requires CONFIG_STM32_SPI1 in the configuration"
#endif

#ifndef CONFIG_SPI_CMDDATA
#  error "The OLED driver requires CONFIG_SPI_CMDDATA in the configuration"
#endif

/* Pin Configuration ********************************************************/

/* UG-2864HSWEG01 OLED Display (SPI 4-wire):
 *
 * --------------------------+----------------------------------------------
 * Connector CON10 J1:      | STM32F4Discovery
 * --------------+-----------+----------------------------------------------
 * CON10 J1:     | CON20 J2: | P1/P2:
 * --------------+-----------+----------------------------------------------
 * 1  3v3        | 3,4 3v3   | P2 3V
 * 3  /RESET     | 8 /RESET  | P2 PB6 (Arbitrary selection)
 * 5  /CS        | 7 /CS     | P2 PB7 (Arbitrary selection)(2)
 * 7  D/C        | 9 D/C     | P2 PB8 (Arbitrary selection)(2)
 * 9  LED+ (N/C) | -----     | -----
 * 2  5V Vcc     | 1,2 Vcc   | P2 5V
 * 4  DI         | 18 D1/SI  | P1 PA7 (GPIO_SPI1_MOSI==GPIO_SPI1_MOSI_1 (1))
 * 6  SCLK       | 19 D0/SCL | P1 PA5 (GPIO_SPI1_SCK==GPIO_SPI1_SCK_1 (2))
 * 8  LED- (N/C) | -----     | ------
 * 10 GND        | 20 GND    | P2 GND
 * --------------+-----------+----------------------------------------------
 * (1) Required because of on-board MEMS
 * (2) Note that the OLED CS and D/C are managed in the stm32_spi.c file.
 * -------------------------------------------------------------------------
 */

/* Definitions in stm32f4discovery.h */

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
  if (!spi)
    {
      lcderr("ERROR: Failed to initialize SPI port 1\n");
    }
  else
    {
      /* Bind the SPI port to the OLED */

      dev = ssd1306_initialize(spi, NULL, devno);
      if (!dev)
        {
          lcderr("ERROR: Failed to bind SPI port 1 to OLED %d\n", devno);
        }
     else
        {
          lcdinfo("Bound SPI port 1 to OLED %d\n", devno);

          /* And turn the OLED on */

          dev->setpower(dev, CONFIG_LCD_MAXPOWER);
          return dev;
        }
    }

  return NULL;
}
#endif /* CONFIG_LCD_UG2864AMBAG01 */
