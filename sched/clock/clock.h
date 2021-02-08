/****************************************************************************
 * sched/clock/clock.h
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

#ifndef __SCHED_CLOCK_CLOCK_H
#define __SCHED_CLOCK_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If CONFIG_SYSTEM_TIME64 is selected and the CPU supports long long types,
 * then a 64-bit system time will be used.
 */

#ifndef CONFIG_HAVE_LONG_LONG
#  undef CONFIG_SYSTEM_TIME64
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if !defined(CONFIG_SCHED_TICKLESS) && !defined(__HAVE_KERNEL_GLOBALS)
  /* The system clock exists (CONFIG_SCHED_TICKLESS), but it not prototyped
   * globally in include/nuttx/clock.h.
   */

#  ifdef CONFIG_SYSTEM_TIME64
extern volatile uint64_t g_system_timer;
#  else
extern volatile uint32_t g_system_timer;
#  endif
#endif

#ifndef CONFIG_CLOCK_TIMEKEEPING
extern struct timespec   g_basetime;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  clock_basetime(FAR struct timespec *tp);

void weak_function clock_initialize(void);
#ifndef CONFIG_SCHED_TICKLESS
void weak_function clock_timer(void);
#endif

int  clock_abstime2ticks(clockid_t clockid,
                         FAR const struct timespec *abstime,
                         FAR sclock_t *ticks);
int  clock_time2ticks(FAR const struct timespec *reltime,
                      FAR sclock_t *ticks);
int  clock_ticks2time(sclock_t ticks, FAR struct timespec *reltime);

#endif /* __SCHED_CLOCK_CLOCK_H */
