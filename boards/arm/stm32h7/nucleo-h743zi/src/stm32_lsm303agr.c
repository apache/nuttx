/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_lsm303agr.c
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
#include <nuttx/arch.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include "stm32.h"
#include <nucleo-h743zi.h>
#include <nuttx/sensors/lsm303agr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32H7_I2C1
#  error "LSM303AGR driver requires CONFIG_STM32H7_I2C1 to be enabled"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_lsm303agr_initialize
 *
 * Description:
 *   Initialize I2C-based LSM303AGR.
 *
 ****************************************************************************/

int stm32_lsm303agr_initialize(char *devpath)
{
  struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("INFO: Initializing LMS303AGR sensor over I2C\n");

#if defined(CONFIG_STM32H7_I2C1)
  i2c = stm32_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  ret = lsm303agr_sensor_register("/dev/lsm303agr0", i2c,
                                  LSM303AGRMAGNETO_ADDR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS303AGR magneto driver %s\n",
            devpath);
      return -ENODEV;
    }

  sninfo("INFO: LMS303AGR sensor has been initialized successfully\n");
#endif

  return ret;
}
