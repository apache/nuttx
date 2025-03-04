/****************************************************************************
 * sched/wdog/wd_gettime.c
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

#include <nuttx/wdog.h>
#include <nuttx/irq.h>

#include "wdog/wdog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_gettime
 *
 * Description:
 *   This function returns the time remaining before the specified watchdog
 *   timer expires.
 *
 * Input Parameters:
 *   wdog - watchdog ID
 *
 * Returned Value:
 *   The time in system ticks remaining until the watchdog time expires.
 *   Zero means either that wdog is not valid or that the wdog has already
 *   expired.
 *
 ****************************************************************************/

sclock_t wd_gettime(FAR struct wdog_s *wdog)
{
  sclock_t delay;

  if (wdog == NULL || !WDOG_ISACTIVE(wdog))
    {
      return 0;
    }

  delay = wdog->expired - clock_systime_ticks();

  return delay < 0 ? 0 : delay;
}
