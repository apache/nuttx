/****************************************************************************
 * boards/risc-v/esp32c3/common/src/esp_board_rng90.c
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

#include <nuttx/arch.h>
#include <nuttx/crypto/rng90.h>
#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>

#ifndef CONFIG_ESPRESSIF_I2C_BITBANG
#  include "espressif/esp_i2c.h"
#else
#  include "espressif/esp_i2c_bitbang.h"
#endif

#include "esp_board_rng90.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_rng90_initialize
 *
 * Description:
 *   Initialize and register the RNG90 True Random Number Generator driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/rngN
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 ****************************************************************************/

int board_rng90_initialize(int devno)
{
  struct i2c_master_s *i2c;
  char devpath[16];
  int ret;

  sninfo("Initializing RNG90\n");

  /* Initialize I2C */

#ifndef CONFIG_ESPRESSIF_I2C_BITBANG
  i2c = esp_i2cbus_initialize(ESPRESSIF_I2C0);
#else
  i2c = esp_i2cbus_bitbang_initialize();
#endif

  if (i2c == NULL)
    {
      snerr("ERROR: Failed to initialize I2C for RNG90\n");
      return -ENODEV;
    }

  /* Register the RNG90 driver at "/dev/rngN" */

  snprintf(devpath, sizeof(devpath), "/dev/rng%d", devno);
  ret = rng90_register(devpath, i2c, RNG90_I2C_ADDR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register RNG90 driver: %d\n", ret);
      return ret;
    }

  return OK;
}

