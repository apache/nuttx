/****************************************************************************
 * libs/libc/sched/clock_ticks2time.c
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
#include <nuttx/clock.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_ticks2time
 *
 * Description:
 *   Convert the system time tick value to a relative time.
 *
 * Input Parameters:
 *   ticks - The number of system time ticks to convert.
 *   reltime - Return the converted system time here.
 *
 * Returned Value:
 *   Always returns OK
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_ticks2time(sclock_t ticks, FAR struct timespec *reltime)
{
  sclock_t remainder;

  reltime->tv_sec  = ticks / TICK_PER_SEC;
  remainder        = ticks - TICK_PER_SEC * reltime->tv_sec;
  reltime->tv_nsec = remainder * NSEC_PER_TICK;
  return OK;
}
