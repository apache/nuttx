/****************************************************************************
 * boards/arm/s32k1xx/rddrone-bms772/src/s32k1xx_ssd1306.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <nuttx/board.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>

#include "s32k1xx_lpi2c.h"

#if defined(CONFIG_I2C) && defined(CONFIG_S32K1XX_LPI2C0) && \
    defined(CONFIG_LCD_SSD1306_I2C)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s *s32k1xx_ssd1306_dev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct i2c_master_s *lpi2c0;
  int ret = OK;

  /* Initialize LPI2C0 */

  lpi2c0 = s32k1xx_i2cbus_initialize(0);
  if (lpi2c0 == NULL)
    {
      i2cerr("ERROR: Failed to initialize LPI2C0\n");
      return -ENODEV;
    }

  /* Initialize SSD1306 on LPI2C0 */

  lcdinfo("Initialize SSD1306 on LPI2C0\n");

  s32k1xx_ssd1306_dev = ssd1306_initialize(lpi2c0, NULL, 0);
  if (s32k1xx_ssd1306_dev == NULL)
    {
      lcderr("ERROR: Failed to initialize SSD1306 on LPI2C0\n");
      s32k1xx_i2cbus_uninitialize(lpi2c0);
      return -ENODEV;
    }

  lcdinfo("Successfully initialized SSD1306 on LPI2C0\n");

  return ret;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  /* Only one display device with devno 0 is supported */

  if (devno == 0)
    {
      return s32k1xx_ssd1306_dev;
    }

  return NULL;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn off the power to uninitialize */

  s32k1xx_ssd1306_dev->setpower(s32k1xx_ssd1306_dev, 0);
}

#endif /* CONFIG_I2C && CONFIG_S32K1XX_LPI2C0 && CONFIG_LCD_SSD1306_I2C */
