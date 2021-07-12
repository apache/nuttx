/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_ht16k33.c
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
#include <nuttx/lcd/ht16k33.h>

#include "esp32_board_i2c.h"
#include "esp32_i2c.h"
#include "esp32_ht16k33.h"

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
 * Name: board_ht16k33_initialize
 *
 * Description:
 *   Initialize and register the BMP180 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/pressN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ht16k33_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing HT16K33!\n");

  /* Initialize I2C Bus */

  i2c = esp32_i2cbus_initialize(busno);

  if (i2c)
    {
      /* Register the HT16K33 Driver at the specified location. */

      ret = ht16k33_register(devno, i2c);
      if (ret < 0)
        {
          lcderr("ERROR: ht16k33_register(%d) failed: %d\n", devno, ret);
          return ret;
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}

