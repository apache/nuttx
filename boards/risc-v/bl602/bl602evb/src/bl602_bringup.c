/****************************************************************************
 * boards/risc-v/bl602/evb/src/bl602_bringup.c
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
#include <nuttx/timers/oneshot.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/input/buttons.h>
#include <bl602_tim_lowerhalf.h>
#include <bl602_oneshot_lowerhalf.h>

#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_bringup
 ****************************************************************************/

int bl602_bringup(void)
{
#if defined(CONFIG_TIMER) && defined(CONFIG_ONESHOT) && \
  defined(CONFIG_BL602_TIMER1)
  struct oneshot_lowerhalf_s *os = NULL;
#endif
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(
        LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
      return ret;
    }
#endif

#if defined(CONFIG_TIMER)
#if defined(CONFIG_BL602_TIMER0)
  ret = bl602_timer_initialize("/dev/timer0", 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize /dev/timer0 Driver: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_BL602_TIMER1) && !defined(CONFIG_ONESHOT)
  ret = bl602_timer_initialize("/dev/timer1", 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize /dev/timer1 Driver: %d\n", ret);
      return ret;
    }
#elif defined(CONFIG_BL602_TIMER1) && defined(CONFIG_ONESHOT)
  os = oneshot_initialize(1, 1);
  if (os == NULL)
    {
      syslog(LOG_ERR, "ERROR: oneshot_initialize failed\n");
    }
  else
    {
#ifdef CONFIG_CPULOAD_ONESHOT
      /* Configure the oneshot timer to support CPU load measurement */

      nxsched_oneshot_extclk(os);

#else
      ret = oneshot_register("/dev/oneshot", os);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to register oneshot at /dev/oneshot: %d\n",
                 ret);
        }
#endif
    }
#endif
#endif

  return ret;
}
