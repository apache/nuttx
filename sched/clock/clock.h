/****************************************************************************
 * sched/clock/clock.h
 *
 *   Copyright (C) 2007-2009, 2014, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#ifdef CONFIG_CLOCK_MONOTONIC
extern struct timespec   g_monotonic_basetime;
#endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
