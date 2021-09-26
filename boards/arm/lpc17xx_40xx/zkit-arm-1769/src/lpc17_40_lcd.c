/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/src/lpc17_40_lcd.c
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
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7567.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc17_40_gpio.h"
#include "lpc17_40_ssp.h"
#include "zkit-arm-1769.h"

#ifdef CONFIG_NX_LCDDRIVER

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
  lpc17_40_configgpio(ZKITARM_OLED_RST);
  lpc17_40_configgpio(ZKITARM_OLED_RS);
  lpc17_40_gpiowrite(ZKITARM_OLED_RST, 1);
  lpc17_40_gpiowrite(ZKITARM_OLED_RS, 1);

  zkit_sspdev_initialize();
  g_spidev = lpc17_40_sspbus_initialize(0);
  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SSP port 0\n");
      return -ENODEV;
    }

  lpc17_40_gpiowrite(ZKITARM_OLED_RST, 0);
  up_mdelay(1);
  lpc17_40_gpiowrite(ZKITARM_OLED_RST, 1);
  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  g_lcddev = st7567_initialize(g_spidev, lcddev);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SSI port 0 to OLCD %d\n", lcddev);
    }
  else
    {
      lcdinfo("Bound SSI port 0 to OLCD %d\n", lcddev);

      /* And turn the OLCD on (CONFIG_LCD_MAXPOWER should be 1) */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
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

/****************************************************************************
 * Name:  lpc17_40_spicmddata
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

int lpc17_40_ssp0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      /* Set GPIO to 1 for data, 0 for command */

      lpc17_40_gpiowrite(ZKITARM_OLED_RS, !cmd);
      return OK;
    }

  return -ENODEV;
}

#endif /* CONFIG_NX_LCDDRIVER */
