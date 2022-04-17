/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_lcd_backpack.c
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

#include <errno.h>
#include <debug.h>
#include <syslog.h>
#include <stdio.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/pcf8574_lcd_backpack.h>

#include "esp32_board_i2c.h"
#include "esp32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_backpack_init
 *
 * Description:
 *   Initialize the LCD1602 display controlled by Backpack with PCF8574
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/slcd0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lcd_backpack_init(int devno, int busno, int rows, int cols)
{
  struct pcf8574_lcd_backpack_config_s cfg =
             LCD_I2C_BACKPACK_CFG_SAINSMART;
  struct i2c_master_s *i2c;
  char devpath[12];
  int ret;

  /* Setup the LCD row and cols size.
   * Note: We are using the LCD_I2C_BACKPACK_CFG_SAINSMART config that
   *       defined the I2C Address to 0x27 to PCF8574. Double check if all
   *       the bits (pins) from PCF8574 connected to the LCD controller
   *       are correct with this LCD CFG definition.
   */

  cfg.rows = rows;
  cfg.cols = cols;

  /* Initialize the I2C1 */

  i2c = esp32_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Register the Segment LCD */

  snprintf(devpath, 12, "/dev/slcd%d", devno);
  ret = pcf8574_lcd_backpack_register(devpath, i2c, &cfg);
  if (ret < 0)
    {
      lcderr("ERROR: pcf8574_lcd_backpack_register(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}
