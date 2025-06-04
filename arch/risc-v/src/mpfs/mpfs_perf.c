/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_perf.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this args for additional information regarding copyright ownership.  The
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
#include <nuttx/clock.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <time.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "hardware/mpfs_memorymap.h"
#include "hardware/mpfs_clint.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_perf_init
 ****************************************************************************/

void up_perf_init(void *arg)
{
}

/****************************************************************************
 * Name: up_perf_gettime
 ****************************************************************************/

clock_t up_perf_gettime(void)
{
  return getreg64(MPFS_CLINT_MTIME);
}

/****************************************************************************
 * Name: up_perf_getfreq
 ****************************************************************************/

unsigned long up_perf_getfreq(void)
{
  return MPFS_MSS_RTC_TOGGLE_CLK;
}

/****************************************************************************
 * Name: up_perf_convert
 ****************************************************************************/

void up_perf_convert(clock_t elapsed, struct timespec *ts)
{
  ts->tv_sec  = elapsed / MPFS_MSS_RTC_TOGGLE_CLK;
  elapsed    -= ts->tv_sec * MPFS_MSS_RTC_TOGGLE_CLK;
  ts->tv_nsec = elapsed * NSEC_PER_SEC / MPFS_MSS_RTC_TOGGLE_CLK;
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay for the requested number of microseconds.
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  clock_t start = up_perf_gettime();
  clock_t end = microseconds * up_perf_getfreq() / USEC_PER_SEC + start + 1;
  while (((sclock_t)(up_perf_gettime() - end)) < 0)
    {
    }
}

/****************************************************************************
 * Name: up_ndelay
 *
 * Description:
 *   Delay for the requested number of nanoseconds.
 *
 ****************************************************************************/

void up_ndelay(unsigned long nanoseconds)
{
  up_udelay((nanoseconds + NSEC_PER_USEC - 1) / NSEC_PER_USEC);
}
