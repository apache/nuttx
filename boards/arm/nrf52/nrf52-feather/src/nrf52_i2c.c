/****************************************************************************
 * boards/arm/nrf52/nrf52-feather/src/nrf52_i2c.c
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
#include <syslog.h>

#ifdef CONFIG_I2C
#  include "nrf52_i2c.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C
int nrf52_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret = OK;

  i2c = nrf52_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          nrf52_i2cbus_uninitialize(i2c);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nrf52_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_I2CTOOL
int nrf52_i2ctool(void)
{
  int ret = OK;
#ifdef CONFIG_NRF52_I2C0_MASTER
  ret = nrf52_i2c_register(0);
  if (ret != OK)
    {
      return ret;
    }
#endif

#ifdef CONFIG_NRF52_I2C1_MASTER
  ret = nrf52_i2c_register(0);
#endif
  return ret;
}
#endif
