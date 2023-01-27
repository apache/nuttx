/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-kaluga-1/src/esp32s2_oneshot.c
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

#include <debug.h>
#include <sys/types.h>
#include <nuttx/timers/timer.h>
#include <nuttx/clock.h>
#include <nuttx/timers/oneshot.h>
#include "esp32s2-kaluga-1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_oneshot_init
 *
 * Description:
 *   Configure the oneshot timer driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_oneshot_init(int timer, uint16_t resolution)
{
  int ret = OK;
  struct oneshot_lowerhalf_s *os_lower = NULL;

  os_lower = oneshot_initialize(timer, resolution);
  if (os_lower != NULL)
    {
#if defined(CONFIG_CPULOAD_ONESHOT)
      /* Configure the oneshot timer to support CPU load measurement */

      nxsched_oneshot_extclk(os_lower);

#else
      ret = oneshot_register("/dev/oneshot", os_lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
            "ERROR: Failed to register oneshot at /dev/oneshot: %d\n", ret);
        }
#endif /* CONFIG_CPULOAD_ONESHOT */
    }
  else
    {
      syslog(LOG_ERR, "ERROR: oneshot_initialize failed\n");
      ret = -EBUSY;
    }

  return ret;
}
