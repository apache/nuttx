/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_bmp280_scu.c
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

#include <nuttx/sensors/bmp280.h>
#include <arch/chip/scu.h>

#include "cxd56_i2c.h"

#ifdef CONFIG_SENSORS_BMP280_SCU_DECI_PRESS
#  define PRESS_NR_SEQS 3
#else
#  define PRESS_NR_SEQS 1
#endif

#ifdef CONFIG_SENSORS_BMP280_SCU_DECI_TEMP
#  define TEMP_NR_SEQS 3
#else
#  define TEMP_NR_SEQS 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP280_SCU
int board_bmp280_initialize(int bus)
{
  int i;
  int ret;
  struct i2c_master_s *i2c;

  sninfo("Initializing BMP280..\n");

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = bmp280_init(i2c, bus);
  if (ret < 0)
    {
      snerr("Error initialize BMP280.\n");
      return ret;
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < PRESS_NR_SEQS; i++)
    {
      ret = bmp280press_register("/dev/press", i, i2c, bus);
      if (ret < 0)
        {
          snerr("Error registering pressure. %d\n", ret);
          return ret;
        }
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < TEMP_NR_SEQS; i++)
    {
      ret = bmp280temp_register("/dev/temp", i, i2c, bus);
      if (ret < 0)
        {
          snerr("Error registering temperature. %d\n", ret);
          return ret;
        }
    }

  return ret;
}

#endif
