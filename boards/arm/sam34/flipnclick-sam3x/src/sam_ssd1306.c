/****************************************************************************
 * boards/arm/sam34/flipnclick-sam3x/src/sam_ssd1306.c
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

/* SSD1306 OLED
 *
 * The HiletGo is a 128x64 OLED that can be driven either via SPI or I2C
 * (SPI is the default and is what is used here).
 * I have mounted the OLED on a proto click board.
 * The OLED is connected as follows:
 *
 * OLED  ALIAS       DESCRIPTION   PROTO CLICK
 * ----- ----------- ------------- -----------------
 *  GND              Ground        GND
 *  VCC              Power Supply  5V  (3-5V)
 *  D0   SCL,CLK,SCK Clock         SCK
 *  D1   SDA,MOSI    Data          MOSI,SDI
 *  RES  RST,RESET   Reset         RST (GPIO OUTPUT)
 *  DC   AO          Data/Command  INT (GPIO OUTPUT)
 *  CS               Chip Select   CS  (GPIO OUTPUT)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>

#if defined(CONFIG_VIDEO_FB) && defined(CONFIG_LCD_FRAMEBUFFER)
#  include <nuttx/video/fb.h>
#endif

#include "sam_gpio.h"
#include "sam_spi.h"

#include "flipnclick-sam3x.h"

#ifdef HAVE_SSD1306

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SPI_CMDDATA
#  error "The OLED driver requires CONFIG_SPI_CMDDATA in the configuration"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_graphics_setup
 *
 * Description:
 *   Called by either NX initialization logic (via board_graphics_setup) or
 *   directly from the board bring-up logic in order to configure the
 *   SSD1306 OLED.
 *
 ****************************************************************************/

struct lcd_dev_s *sam_graphics_setup(unsigned int devno)
{
  struct spi_dev_s *spi;
  struct lcd_dev_s *dev;

  /* Configure the OLED GPIOs. This initial configuration is RESET low,
   * putting the OLED into reset state.
   */

  sam_configgpio(GPIO_SSD1306_RST);

  /* Wait a bit then release the OLED from the reset state */

  up_mdelay(20);
  sam_gpiowrite(GPIO_SSD1306_RST, true);

  /* Get the SPI1 port interface */

  spi = sam_spibus_initialize(GPIO_SSD1306_CS);
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

#if defined(CONFIG_VIDEO_FB) && defined(CONFIG_LCD_FRAMEBUFFER)

          /* Initialize and register the simulated framebuffer driver */

          ret = fb_register(0, 0);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
            }
#endif

          return dev;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: board_graphics_setup
 *
 * Description:
 *   Called by NX initialization logic to configure the OLED.
 *
 ****************************************************************************/

#ifdef CONFIG_NXSTART_EXTERNINIT
struct lcd_dev_s *board_graphics_setup(unsigned int devno)
{
  return sam_graphics_setup(devno);
}
#endif
#endif /* HAVE_SSD1306 */
