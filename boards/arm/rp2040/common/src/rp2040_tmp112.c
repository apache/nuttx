/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_tmp112.c
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

#include <stdio.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sensors/tmp112.h>
#include <nuttx/i2c/i2c_master.h>

#include "rp2040_i2c.h"
#include "rp2040_tmp112.h"

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
 * Name: board_tmp112_initialize
 *
 * Description:
 *   Initialize and register the TMP112 temperature sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/tempN.
 *           The value is 0-indexed internally, but gets logged as 1-indexed.
 *   busno - The I2C bus number.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_tmp112_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  char devpath[11];
  int ret;

  sninfo("Initializing TMP112 #%d on I2C bus %d\n", devno + 1, busno);

  /* Initialize the I2C bus */

  i2c = rp2040_i2cbus_initialize(busno);
  if (i2c)
    {
      /* Then try to register the temperature sensor */

      snprintf(devpath, sizeof(devpath), "/dev/temp%d", devno);
      ret = tmp112_register(devpath, devno, i2c);
      if (ret < 0)
        {
          snerr("ERROR: Error registering TMP112 #%d in I2C%d\n",
                devno + 1, busno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
