/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_st7032.c
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

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/st7032.h>

#include "stm32.h"
#include "stm32f4discovery.h"

#if defined(CONFIG_I2C) && defined(CONFIG_STM32_I2C1) && \
    defined(CONFIG_LCD_ST7032)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ST7032_I2C_PORTNO 1   /* On I2C1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_st7032init
 *
 * Description:
 *   Initialize the st7032 display
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/disp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_st7032init(const char *devpath)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(ST7032_I2C_PORTNO);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Register the ST7032 Driver at the specified location. */

  ret = st7032_register(devpath, i2c);
  if (ret < 0)
    {
      lcderr("ERROR: st7032_register(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_LEDS_ST7032 */
