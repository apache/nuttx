/****************************************************************************
 * boards/arm/stm32u5/nucleo-u5a5zj-q/src/stm32_bringup.c
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

#include <sys/mount.h>
#include <sys/types.h>
#include <debug.h>

#include <nuttx/input/buttons.h>
#include <nuttx/leds/userled.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/board.h>
#include <nuttx/clock.h>

#include "nucleo-u5a5zj-q.h"

#include <arch/board/board.h>

#include <stm32_spi.h>

#if defined(CONFIG_I2C)

#include "stm32_i2c.h"
struct i2c_master_s *i2c1_m;
struct i2c_master_s *i2c2_m;
#  ifdef CONFIG_RTC_DSXXXX
#    include <nuttx/timers/rtc.h>
#    include <nuttx/timers/ds3231.h>
#  endif /* CONFIG_RTC_DSXXXX */

#endif /* CONFIG_I2C */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DEVNO_ZERO   0
#define DEVNO_ONE    1
#define DEVNO_TWO    2

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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C)
  i2c1_m = stm32_i2cbus_initialize(1);
  if (i2c1_m == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init i2c controller\n");
      return(-1);
    }

  ret = i2c_register(i2c1_m, DEVNO_ONE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
            DEVNO_ONE, ret);
      stm32_i2cbus_uninitialize(i2c1_m);
      return -1;
    }

#if defined(STM32U5_I2C2)
  i2c2_m = stm32_i2cbus_initialize(2);
  if (i2c2_m == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to init i2c controller\n");
      return(-1);
    }

  ret = i2c_register(i2c2_m, DEVNO_TWO);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
            DEVNO_TWO, ret);
      stm32_i2cbus_uninitialize(i2c1_m);
      stm32_i2cbus_uninitialize(i2c2_m);
      return -1;
    }
#endif /* STM32_I2C2 */
#endif /* CONFIG_I2C */

#if defined(CONFIG_RTC) && defined(CONFIG_RTC_EXTERNAL) && defined(CONFIG_RTC_DSXXXX)
  ret = dsxxxx_rtc_initialize(i2c1_m);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: dsxxxx_rtc_initialize() failed: %d\n", ret);
    }
  else
    {
      /* Synchronize the system time to the RTC time */

      syslog(LOG_INFO, "INFO: clock sync\n");
      clock_synchronize(NULL);
    }
#endif /* CONFIG_RTC */

  UNUSED(ret);
  return OK;
}
