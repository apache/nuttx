/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_kx022_scu.c
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
#include <errno.h>

#include <nuttx/board.h>

#include <nuttx/sensors/kx022.h>
#include <arch/chip/scu.h>

#include "cxd56_i2c.h"

#ifdef CONFIG_SENSORS_KX022_SCU_DECI
#  define KX022_FIFO_CNT 3
#else
#  define KX022_FIFO_CNT 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_KX022_SCU
int board_kx022_initialize(const char *devpath, int bus)
{
  int fifoid = 0;
  int ret;
  struct i2c_master_s *i2c;

  sninfo("Initializing KX022...\n");

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = kx022_init(i2c, bus);
  if (ret < 0)
    {
      snerr("Error initialize KX022.\n");
      return ret;
    }

  /* Register devices for each FIFOs at I2C bus */

  for (fifoid = 0; fifoid < KX022_FIFO_CNT; fifoid++)
    {
      ret = kx022_register(devpath, fifoid, i2c, bus);
      if (ret < 0)
        {
          snerr("Error registering KX022.\n");
          return ret;
        }
    }

  return ret;
}
#endif /* CONFIG_SENSORS_KX022_SCU */
