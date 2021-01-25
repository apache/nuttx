/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_bh1745nuc_scu.c
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

#include <nuttx/sensors/bh1745nuc.h>
#include <arch/chip/scu.h>

#include "cxd56_i2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BH1745NUC_SCU
int board_bh1745nuc_initialize(FAR const char *devpath, int bus)
{
  int ret;
  FAR struct i2c_master_s *i2c;

  sninfo("Initializing BH1745NUC...\n");

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = bh1745nuc_init(i2c, bus);
  if (ret < 0)
    {
      snerr("Error initialize BH1745NUC.\n");
      return ret;
    }

  /* Register devices for each FIFOs at I2C bus */

  ret = bh1745nuc_register(devpath, 0, i2c, bus);
  if (ret < 0)
    {
      snerr("Error registering BH1745NUC.\n");
      return ret;
    }

  return ret;
}
#endif /* CONFIG_SENSORS_BH1745NUC_SCU */
