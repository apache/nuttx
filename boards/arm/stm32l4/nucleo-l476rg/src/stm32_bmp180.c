/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_bmp180.c
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

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/bmp180.h>

#include "stm32l4.h"
#include "stm32l4_i2c.h"
#include "nucleo-l476rg.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP180)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bmp180initialize
 *
 * Description:
 *   Initialize and register the MPL115A Pressure Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_bmp180initialize(const char *devpath)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing BMP180!\n");

  /* Initialize I2C */

  i2c = stm32l4_i2cbus_initialize(BMP180_I2C_PORTNO);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  ret = bmp180_register(devpath, i2c);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BM180\n");
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP180 */
