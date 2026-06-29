/****************************************************************************
 * boards/arm/rp2040/common/src/rp2040_husb238.c
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
#include <nuttx/power/husb238.h>

#include "rp2040_i2c.h"
#include "rp2040_husb238.h"

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
 * Name: board_husb238_initialize
 *
 * Description:
 *   Initialize and register the HUSB238 USB-C PD sink controller.
 *
 * Input Parameters:
 *   i2c   - An instance of the I2C interface to use.
 *   devno - The device number, used to build the device path as /dev/usbpdN.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_husb238_initialize(FAR struct i2c_master_s *i2c, int devno)
{
  char devpath[12];
  int ret;

  pwrinfo("Initializing HUSB238\n");

  if (i2c)
    {
      snprintf(devpath, sizeof(devpath), "/dev/usbpd%d", devno);
      ret = husb238_register(devpath, i2c);
      if (ret < 0)
        {
          pwrerr("ERROR: Error registering HUSB238 at /dev/usbpd%d\n",
                 devno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
