/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_bmi160_scu.c
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
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/bmi160.h>
#include <arch/chip/scu.h>

#if defined(CONFIG_SENSORS_BMI160_SCU_SPI)
#include "cxd56_spi.h"
#else
#include "cxd56_i2c.h"
#endif

#ifdef CONFIG_SENSORS_BMI160_SCU_DECI_GYRO
#  define GYRO_NR_SEQS 3
#else
#  define GYRO_NR_SEQS 1
#endif

#ifdef CONFIG_SENSORS_BMI160_SCU_DECI_ACCEL
#  define ACCEL_NR_SEQS 3
#else
#  define ACCEL_NR_SEQS 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_SENSORS_BMI160_SCU)
#if defined(CONFIG_SENSORS_BMI160_SCU_SPI)

int board_bmi160_initialize(int bus)
{
  int ret;
  struct spi_dev_s *spi;

  sninfo("Initializing BMI160..\n");

  /* Initialize spi device */

  spi = cxd56_spibus_initialize(bus);
  if (!spi)
    {
      snerr("ERROR: Failed to initialize spi%d.\n", bus);
      return -ENODEV;
    }

  int i;

  ret = bmi160_init(spi);
  if (ret < 0)
    {
      snerr("Error initialize BMI160\n");
      return ret;
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < GYRO_NR_SEQS; i++)
    {
      ret = bmi160gyro_register("/dev/gyro", i, spi);
      if (ret < 0)
        {
          snerr("Error registering gyroscope. %d\n", ret);
          return ret;
        }
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < ACCEL_NR_SEQS; i++)
    {
      ret = bmi160accel_register("/dev/accel", i, spi);
      if (ret < 0)
        {
          snerr("Error registering accelerometer. %d\n", ret);
          return ret;
        }
    }

  return ret;
}

#else /* !CONFIG_SENSORS_BMI160_SCU_SPI */

int board_bmi160_initialize(int bus)
{
  int ret;
  struct i2c_master_s *i2c;

  sninfo("Initializing BMI160..\n");

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  int i;

  ret = bmi160_init(i2c, bus);
  if (ret < 0)
    {
      snerr("Error initialize BMI160\n");
      return ret;
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < GYRO_NR_SEQS; i++)
    {
      ret = bmi160gyro_register("/dev/gyro", i, i2c, bus);
      if (ret < 0)
        {
          snerr("Error registering gyroscope. %d\n", ret);
          return ret;
        }
    }

  /* Create char devices for each FIFOs */

  for (i = 0; i < ACCEL_NR_SEQS; i++)
    {
      ret = bmi160accel_register("/dev/accel", i, i2c, bus);
      if (ret < 0)
        {
          snerr("Error registering accelerometer. %d\n", ret);
          return ret;
        }
    }

  return ret;
}

#endif  /* CONFIG_SENSORS_BMI160_SCU_SPI */
#endif  /* CONFIG_SENSORS_BMI160_SCU */
