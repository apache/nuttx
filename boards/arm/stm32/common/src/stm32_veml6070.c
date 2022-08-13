/****************************************************************************
 * boards/arm/stm32/common/src/stm32_veml6070.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/veml6070.h>

#include "stm32.h"
#include "stm32_i2c.h"

#ifdef CONFIG_SENSORS_VEML6070

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
 * Name: board_veml6070_initialize
 *
 * Description:
 *   Initialize and register the VEML6070 UV-A Light sensor.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as
 *           /dev/uvlightN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_veml6070_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  char devpath[14];
  int ret;

  sninfo("Initializing VEML6070!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the light sensor */

  snprintf(devpath, 14, "/dev/uvlight%d", devno);
  ret = veml6070_register(devpath, i2c, VEML6070_I2C_DATA_LSB_CMD_ADDR);
  if (ret < 0)
    {
      snerr("ERROR: Error registering VEML6070\n");
    }

  return ret;
}

#endif
