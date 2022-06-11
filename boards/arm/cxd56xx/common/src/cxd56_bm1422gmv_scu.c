/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_bm1422gmv_scu.c
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

#include <nuttx/sensors/bm1422gmv.h>
#include <arch/chip/scu.h>

#include "cxd56_i2c.h"

#ifdef CONFIG_SENSORS_BM1422GMV_SCU_DECI
#  define BM1422GMV_PATH_CNT 3
#else
#  define BM1422GMV_PATH_CNT 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BM1422GMV_SCU
int board_bm1422gmv_initialize(const char *devpath, int bus)
{
  int id = 0;
  int ret;
  struct i2c_master_s *i2c;

  sninfo("Initializing BM1422GMV...\n");

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = bm1422gmv_init(i2c, bus);
  if (ret < 0)
    {
      snerr("Error initialize BM1422GMV.\n");
      return ret;
    }

  /* Register devices for each FIFOs at I2C bus */

  for (id = 0; id < BM1422GMV_PATH_CNT; id++)
    {
      ret = bm1422gmv_register(devpath, id, i2c, bus);
      if (ret < 0)
        {
          snerr("Error registering BM1422GMV.\n");
          return ret;
        }
    }

  return ret;
}
#endif /* CONFIG_SENSORS_BM1422GMV_SCU */
