/****************************************************************************
 * boards/arm/stm32/stm32f401rc-rs485/src/stm32_at24.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/mount.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/eeprom/i2c_xx24xx.h>

#include "stm32f401rc-rs485.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AT24_I2C_BUS     1  /* EEPROM chip is configured to use I2C1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_at24_init
 *
 * Description:
 *   Initialize and configure the AT24 serial EEPROM
 *
 ****************************************************************************/

int stm32_at24_init(char *path)
{
  struct i2c_master_s *i2c;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C bus driver */

      finfo("Initialize I2C%d\n", AT24_I2C_BUS);
      i2c = stm32_i2cbus_initialize(AT24_I2C_BUS);
      if (!i2c)
        {
          ferr("ERROR: Failed to initialize I2C%d\n", AT24_I2C_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      finfo("Bind the AT24 EEPROM driver to I2C%d\n", AT24_I2C_BUS);
      ret = ee24xx_initialize(i2c, 0x50, path, EEPROM_AT24CM02, false);
      if (ret < 0)
        {
          ferr("ERROR: Failed to bind I2C%d to the AT24 EEPROM driver\n",
               AT24_I2C_BUS);
          return -ENODEV;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

