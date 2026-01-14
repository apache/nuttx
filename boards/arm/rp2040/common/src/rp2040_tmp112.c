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
 *   i2c   - An instance of the I2C interface to use.
 *   devno - The device number, used to build the device path as /dev/tempN.
 *   addr  - The I2C address to use.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_tmp112_initialize(FAR struct i2c_master_s *i2c, int devno,
                            uint8_t addr)
{
  char devpath[11];
  int ret;

  sninfo("Initializing TMP112 with address %#4x\n", addr);

  if (i2c)
    {
      snprintf(devpath, sizeof(devpath), "/dev/temp%d", devno);
      ret = tmp112_register(devpath, i2c, addr);
      if (ret < 0)
        {
          snerr("ERROR: Error registering TMP112 at /dev/temp%d\n", devno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
