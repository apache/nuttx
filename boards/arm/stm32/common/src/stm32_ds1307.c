/****************************************************************************
 * boards/arm/stm32/common/src/stm32_ds1307.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#if defined(CONFIG_I2C) && defined(CONFIG_RTC_DS1307)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ds1307_initialize
 *
 * Description:
 *   Initialize and configure the DS1307 RTC
 *
 * Input Parameters:
 *   busno - The I2C bus number where DS1307 is connected.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ds1307_initialize(int busno)
{
  struct i2c_master_s *i2c;
  int ret;

  rtcinfo("Initialize I2C%d\n", busno);

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (!i2c)
    {
      rtcerr("ERROR: Failed to initialize I2C%d\n", busno);
      return -ENODEV;
    }

  /* Now bind the I2C interface to the DS1307 RTC driver */

  rtcinfo("Bind the DS1307 RTC driver to I2C%d\n", busno);
  ret = dsxxxx_rtc_initialize(i2c);
  if (ret < 0)
    {
      rtcerr("ERROR: Failed to bind I2C%d to the DS1307 RTC driver\n",
             busno);
      return -ENODEV;
    }

  /* Synchronize the system time to the RTC time */

  clock_synchronize(NULL);

  /* Now we are initialized */

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_RTC_DS1307 */
