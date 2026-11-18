/****************************************************************************
 * libs/libc/sched/clock_perf.c
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
#include <nuttx/seqlock.h>
#include <nuttx/userspace.h>

/* In a flat build g_perf is shared by the application and the kernel.  In a
 * protected build it is the us_perf member of the user-space data block, so
 * the application and the kernel still read and update the same state.
 */

#if defined(CONFIG_PERF_OVERFLOW_CORRECTION) && !defined(CONFIG_SMP)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t noinstrument_function perf_gettime(void)
{
  clock_t overflow;
  clock_t result;
  uint32_t seq;

  do
    {
      seq = read_seqbegin(&g_perf.lock);
      clock_t now = up_perf_gettime();

      if (now < g_perf.last)
        {
          overflow = g_perf.overflow + 1;
        }
      else
        {
          overflow = g_perf.overflow;
        }

      result = now | (overflow << CONFIG_ARCH_PERF_COUNT_BITWIDTH);
    }
  while (read_seqretry(&g_perf.lock, seq));

  return result;
}

#elif !defined(CONFIG_BUILD_FLAT) && \
  defined(CONFIG_ARCH_PERF_EVENTS_USER_ACCESS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * perf_gettime
 ****************************************************************************/

clock_t noinstrument_function perf_gettime(void)
{
  return up_perf_gettime();
}

#endif
