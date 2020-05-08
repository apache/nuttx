/****************************************************************************
 * boards/arm/stm32/common/src/stm32_ssd1306.c
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
#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct lcd_dev_s    *g_lcddev;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ssd1306_initialize
 *
 * Description:
 *   Initialize and register the device
 *
 * Input Parameters:
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ssd1306_initialize(int busno)
{
  FAR struct i2c_master_s* i2c;

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (!i2c)
    {
      lcderr("ERROR: Failed to initialize I2C port %d\n", OLED_I2C_PORT);
      return -ENODEV;
    }

  /* Bind the I2C port to the OLED */

  g_lcddev = ssd1306_initialize(i2c, NULL, 0);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind I2C port 1 to OLED %d: %d\n", devno);
      return -ENODEV;
    }
  else
    {
      lcdinfo("Bound I2C port %d to OLED %d\n", OLED_I2C_PORT, devno);

      /* And turn the OLED on */

      g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
      return OK;
    }
}

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

FAR struct lcd_dev_s *board_ssd1306_getdev(void)
{
  return g_lcddev;
}

