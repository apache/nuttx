/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_ssd1306.c
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

#include "stm32.h"
#include "nucleo-h743zi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct i2c_master_s *g_i2c;
FAR struct lcd_dev_s    *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  /* Initialize I2C */

  g_i2c = stm32_i2cbus_initialize(OLED_I2C_PORT);
  if (!g_i2c)
    {
      lcderr("ERROR: Failed to initialize I2C port %d\n", OLED_I2C_PORT);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int devno)
{
  /* Bind the I2C port to the OLED */

  g_lcddev = ssd1306_initialize(g_i2c, NULL, devno);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind I2C port 1 to OLED %d: %d\n", devno);
    }
  else
    {
      lcdinfo("Bound I2C port %d to OLED %d\n", OLED_I2C_PORT, devno);

      /* And turn the OLED on */

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
