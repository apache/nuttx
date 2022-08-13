/****************************************************************************
 * boards/arm/tiva/dk-tm4c129x/src/tm4c_tmp100.c
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

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lm75.h>

#include "tiva_i2c.h"
#include "dk-tm4c129x.h"

#if defined(CONFIG_I2C) && defined(CONFIG_LM75_I2C) && \
    defined(CONFIG_TIVA_I2C6)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_tmp100_initialize
 *
 * Description:
 *   Initialize and register the LM-75 Temperature Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tiva_tmp100_initialize(const char *devpath)
{
  struct i2c_master_s *i2c;
  int ret;

  /* Get an instance of the I2C6 interface */

  i2c =  tiva_i2cbus_initialize(TMP100_I2CBUS);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the temperature sensor */

  ret = lm75_register(devpath, i2c, TMP100_I2CADDR);
  if (ret < 0)
    {
      tiva_i2cbus_uninitialize(i2c);
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_LM75_I2C && CONFIG_TIVA_I2C6 */
