/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_ds1307.c
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

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/timers/ds3231.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "stm32f4discovery.h"

#if defined(CONFIG_I2C) && defined(CONFIG_RTC_DS1307)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DS1307_I2C_ADDR    0x6f /* DS1307 I2C Address */
#define DS1307_I2C_BUS     1    /* DS1307 is on I2C1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ds1307_init
 *
 * Description:
 *   Initialize and configure the DS1307 RTC
 *
 ****************************************************************************/

int stm32_ds1307_init(void)
{
  struct i2c_master_s *i2c;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the I2C bus driver */

      rtcinfo("Initialize I2C%d\n", DS1307_I2C_BUS);
      i2c = stm32_i2cbus_initialize(DS1307_I2C_BUS);
      if (!i2c)
        {
          rtcerr("ERROR: Failed to initialize I2C%d\n", DS1307_I2C_BUS);
          return -ENODEV;
        }

      /* Now bind the I2C interface to the DS1307 RTC driver */

      rtcinfo("Bind the DS1307 RTC driver to I2C%d\n", DS1307_I2C_BUS);
      ret = dsxxxx_rtc_initialize(i2c);
      if (ret < 0)
        {
          rtcerr("ERROR: Failed to bind I2C%d to the DS1307 RTC driver\n",
                 DS1307_I2C_BUS);
          return -ENODEV;
        }

#ifdef CONFIG_I2C_DRIVER
      /* Register the I2C to get the "nsh> i2c bus" command working */

      ret = i2c_register(i2c, DS1307_I2C_BUS);
      if (ret < 0)
        {
          rtcerr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          return -ENODEV;
        }
#endif

      /* Synchronize the system time to the RTC time */

      clock_synchronize(NULL);

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_RTC_DS1307 */
