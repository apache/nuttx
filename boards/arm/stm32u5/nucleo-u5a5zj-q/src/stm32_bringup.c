/****************************************************************************
 * boards/arm/stm32u5/nucleo-u5a5zj-q/src/stm32_bringup.c
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

#include <sys/types.h>
#include <nuttx/debug.h>

#include <nuttx/input/buttons.h>
#include <nuttx/leds/userled.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/board.h>
#include <nuttx/clock.h>

#include "nucleo-u5a5zj-q.h"

#include <arch/board/board.h>

#include <stm32_spi.h>

#ifdef CONFIG_RTC_DSXXXX
#  include <nuttx/timers/rtc.h>
#  include <nuttx/timers/ds3231.h>
#  include "stm32_i2c.h"
#endif

#include "board_common.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Should not be here, but there is a bug in clock_initalize logic that
 * prevents external RTC to be used when no internal RTC !
 * **************************************************************************/
#if defined(CONFIG_RTC) && defined(CONFIG_RTC_EXTERNAL)
int up_rtc_initialize(void)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#ifdef CONFIG_RTC_DSXXXX
  struct i2c_master_s *i2c;
#endif
  int ret;

  /* Initialize the board features common to all STM32 boards */

  ret = stm32_common_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_common_initialize() failed: %d\n",
             ret);
    }

#ifdef CONFIG_RTC_DSXXXX
  i2c = stm32_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C1 interface\n");
    }
  else
    {
      ret = dsxxxx_rtc_initialize(i2c);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: dsxxxx_rtc_initialize() failed: %d\n",
                 ret);
        }
      else
        {
          /* Synchronize the system time to the RTC time */

          clock_synchronize(NULL);
        }
    }
#endif

  UNUSED(ret);
  return OK;
}
