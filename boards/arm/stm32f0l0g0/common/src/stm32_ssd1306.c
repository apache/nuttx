/****************************************************************************
 * boards/arm/stm32f0l0g0/common/src/stm32_ssd1306.c
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
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>

#include "stm32_i2c.h"
#include "stm32_spi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s    *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ssd1306_initialize
 *
 * Description:
 *   Initialize and register the device. I2C version.
 *
 * Input Parameters:
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
#ifdef CONFIG_LCD_SSD1306_I2C
int board_ssd1306_initialize(int busno)
{
  struct i2c_master_s *i2c;
  const int devno = 0;

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (!i2c)
    {
      lcderr("ERROR: Failed to initialize I2C port %d\n", busno);
      return -ENODEV;
    }

  /* Bind the I2C port to the OLED */

  g_lcddev = ssd1306_initialize(i2c, NULL, devno);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind I2C port %d to OLED %d\n", busno, devno);
      return -ENODEV;
    }
  else
    {
      lcdinfo("Bound I2C port %d to OLED %d\n", busno, devno);

      /* And turn the OLED on */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
      return OK;
    }
}
#endif

/****************************************************************************
 * Name: board_ssd1306_initialize
 *
 * Description:
 *   Initialize and register the device. SPI version.
 *
 * Input Parameters:
 *   busno - The SPI bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
#ifdef CONFIG_LCD_SSD1306_SPI
int board_ssd1306_initialize(int busno)
{
  struct spi_dev_s *spi;
  const int devno = 0;

  /* Initialize SPI */

  spi = stm32_spibus_initialize(busno);
  if (!spi)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", busno);
      return -ENODEV;
    }

  /* Bind the SPI port to the OLED */

  g_lcddev = ssd1306_initialize(spi, NULL, devno);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind SPI port %d to OLED %d\n", busno, devno);
      return -ENODEV;
    }
  else
    {
      lcdinfo("Bound SPI port %d to OLED %d\n", busno, devno);

      /* And turn the OLED on */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);

      ssd1306_fill(g_lcddev, 0xff);

      return OK;
    }
}
#endif

/****************************************************************************
 * Name: board_ssd1306_getdev
 *
 * Description:
 *   Get the SSD1306 device driver instance
 *
 * Returned Value:
 *   Pointer to the instance
 *
 ****************************************************************************/

struct lcd_dev_s *board_ssd1306_getdev(void)
{
  return g_lcddev;
}
