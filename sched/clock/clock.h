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

extern volatile clock_t g_system_ticks;
#endif

#ifndef CONFIG_CLOCK_TIMEKEEPING
extern struct timespec  g_basetime;
#endif

#ifdef CONFIG_CLOCK_ADJTIME
extern long long g_clk_adj_usec;
extern long long g_clk_adj_count;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  clock_basetime(FAR struct timespec *tp);

void clock_initialize(void);
#ifndef CONFIG_SCHED_TICKLESS
void clock_timer(void);
#else
#  define clock_timer()
#endif

#ifdef CONFIG_CLOCK_ADJTIME
void clock_set_adjust(long long adj_usec, long long adj_count,
                      long long *adj_usec_old, long long *adj_count_old);
#endif

int  clock_abstime2ticks(clockid_t clockid,
                         FAR const struct timespec *abstime,
                         FAR sclock_t *ticks);

#endif /* __SCHED_CLOCK_CLOCK_H */
