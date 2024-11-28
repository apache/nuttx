/****************************************************************************
 * arch/x86_64/src/intel64/intel64_perf.c
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

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include "x86_64_internal.h"

#ifdef CONFIG_ARCH_PERF_EVENTS

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern unsigned long g_x86_64_timer_freq;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_perf_init(void *arg)
{
  /* The default tsc will be turned on when the system starts */

  UNUSED(arg);
}

unsigned long up_perf_getfreq(void)
{
  return g_x86_64_timer_freq;
}

clock_t up_perf_gettime(void)
{
  return rdtscp();
}

void up_perf_convert(clock_t elapsed, struct timespec *ts)
{
  clock_t left;

  ts->tv_sec  = elapsed / g_x86_64_timer_freq;
  left        = elapsed - ts->tv_sec * g_x86_64_timer_freq;
  ts->tv_nsec = NSEC_PER_SEC * (uint64_t)left / g_x86_64_timer_freq;
}
#endif

