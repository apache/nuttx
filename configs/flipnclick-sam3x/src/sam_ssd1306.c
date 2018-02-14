/****************************************************************************
 * config/flipnclick-sam3x/src/sam_ssd1306.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

/* SSD1306 OLED
 *
 * The HiletGo is a 128x64 OLED that can be driven either via SPI or I2C (SPI
 * is the default and is what is used here).  I have mounted the OLED on a
 * proto click board.  The OLED is connected as follows:
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

FAR struct lcd_dev_s *sam_graphics_setup(unsigned int devno)
{
  FAR struct spi_dev_s *spi;
  FAR struct lcd_dev_s *dev;

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
          lcderr("ERROR: Failed to bind SPI port 1 to OLED %d: %d\n", devno);
        }
     else
        {
          lcdinfo("Bound SPI port 1 to OLED %d\n", devno);

          /* And turn the OLED on */

          (void)dev->setpower(dev, CONFIG_LCD_MAXPOWER);

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
FAR struct lcd_dev_s *board_graphics_setup(unsigned int devno)
{
  return sam_graphics_setup(devno);
}
#endif

#endif /* HAVE_SSD1306 */
