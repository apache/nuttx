/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_ssd1306.c
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

#include "rp2040_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OLED_I2C_PORT         0 /* OLED display connected to I2C0 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s    *g_lcddev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct i2c_master_s *i2c;
  const int busno = OLED_I2C_PORT;
  const int devno = 0;

  /* Initialize I2C */

  i2c = rp2040_i2cbus_initialize(busno);
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

/****************************************************************************
 * Name:  board_lcd_getdev
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev == 0)
    {
      return g_lcddev;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
}
