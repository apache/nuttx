/****************************************************************************
 * sched/clock/clock_realtime2absticks.c
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

#include <time.h>
#include <errno.h>
#include <debug.h>
#include "clock/clock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_realtime2absticks
 *
 * Description:
 *   Convert real time to monotonic ticks.
 *
 * Input Parameters:
 *   mono - Return the converted time here.
 *   abstime - Convert this absolute time to ticks
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Interrupts should be disabled so that the time is not changing during
 *   the calculation
 *
 ****************************************************************************/

int clock_realtime2absticks(FAR const struct timespec *reltime,
                            FAR clock_t *absticks)
{
#ifndef CONFIG_CLOCK_TIMEKEEPING
  struct timespec mono;

  clock_timespec_subtract(reltime, &g_basetime, &mono);

  *absticks = clock_time2ticks(&mono);

  return OK;
#else
  return -ENOSYS;
#endif
}
