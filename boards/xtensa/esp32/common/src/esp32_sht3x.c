/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_sht3x.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sensors/sht3x.h>
#include <nuttx/i2c/i2c_master.h>

#include "esp32_board_i2c.h"
#include "esp32_i2c.h"
#include "esp32_sht3x.h"

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

#define SHT3X_I2CADDR    0x44

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sht3x_initialize
 *
 * Description:
 *   Initialize and register the SHT3X Temperature/Humidity Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/tempN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_sht3x_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  char devpath[12];
  int ret;

  sninfo("Initializing SHT3X!\n");

  /* Initialize SHT3X */

  i2c = esp32_i2cbus_initialize(busno);

  if (i2c)
    {
      /* Then try to register the sensor in I2C Bus */

      snprintf(devpath, 12, "/dev/temp%d", devno);
      ret = sht3x_register(devpath, i2c, SHT3X_I2CADDR);
      if (ret < 0)
        {
          snerr("ERROR: Error registering SHT3X in I2C%d\n", busno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}

