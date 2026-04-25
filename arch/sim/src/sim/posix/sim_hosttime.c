/****************************************************************************
 * arch/sim/src/sim/posix/sim_hosttime.c
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

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "sim_internal.h"

#ifdef __APPLE__
#  include <dispatch/dispatch.h>
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t g_start;

/* Ratio of simulated time to real time in percent.  100 means real-time
 * (default).  Values > 100 speed up simulated time; values < 100 slow it
 * down.  Overridable at runtime via --sim-rt-ratio=<percent>.
 */

static int g_time_ratio = CONFIG_SIM_WALLTIME_RATIO;
#ifdef __APPLE__
static dispatch_source_t g_timer;

static void host_timer_handler(void *context)
{
  raise(SIGALRM);
}
#else
static timer_t g_timer;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_inittimer
 ****************************************************************************/

int host_inittimer(void)
{
  struct timespec tp;

  clock_gettime(CLOCK_MONOTONIC, &tp);

  g_start = 1000000000ull * tp.tv_sec + tp.tv_nsec;

#ifdef __APPLE__
  g_timer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0,
                                   dispatch_get_global_queue(
                                     DISPATCH_QUEUE_PRIORITY_HIGH, 0));
  if (g_timer == NULL)
    {
      return -1;
    }

  dispatch_source_set_event_handler_f(g_timer, host_timer_handler);
  dispatch_resume(g_timer);
  return 0;
#else
  struct sigevent sigev =
    {
      0
    };

  sigev.sigev_notify = SIGEV_SIGNAL;
  sigev.sigev_signo  = SIGALRM;

  return timer_create(CLOCK_MONOTONIC, &sigev, &g_timer);
#endif
}

/****************************************************************************
 * Name: host_gettime
 ****************************************************************************/

uint64_t host_gettime(bool rtc)
{
  struct timespec tp;
  uint64_t current;

  clock_gettime(rtc ? CLOCK_REALTIME : CLOCK_MONOTONIC, &tp);
  current = 1000000000ull * tp.tv_sec + tp.tv_nsec;

  if (rtc)
    {
      return current;
    }

  /* Apply time ratio: simulated_time = real_elapsed * ratio / 100 */

  return ((current - g_start) * g_time_ratio) / 100;
}

/****************************************************************************
 * Name: host_sleep
 ****************************************************************************/

void host_sleep(uint64_t nsec)
{
  usleep((nsec + 999) / 1000);
}

/****************************************************************************
 * Name: host_sleepuntil
 ****************************************************************************/

void host_sleepuntil(uint64_t nsec)
{
  uint64_t now;

  now = host_gettime(false);
  if (nsec > now + 1000)
    {
      /* nsec is in simulated time; convert back to real duration to sleep */

      usleep((((nsec - now) * 100) / g_time_ratio) / 1000);
    }
}

/****************************************************************************
 * Name: host_set_timeratio
 *
 * Description:
 *   Set the ratio of simulated time to real time in percent.  100 (default)
 *   means simulated time advances at the same rate as real time.  Values
 *   greater than 100 speed up simulated time; values less than 100 slow it
 *   down.
 *
 * Input Parameters:
 *   ratio - The new time ratio in percent (must be > 0)
 *
 ****************************************************************************/

void host_set_timeratio(int ratio)
{
  if (ratio > 0)
    {
      g_time_ratio = ratio;
    }
}

/****************************************************************************
 * Name: host_settimer
 *
 * Description:
 *   Set up a timer to send periodic signals.
 *
 * Input Parameters:
 *   nsec - timer expire time
 *
 * Returned Value:
 *   On success, (0) zero value is returned, otherwise a negative value.
 *
 ****************************************************************************/

int host_settimer(uint64_t nsec)
{
  /* nsec is in simulated time; convert back to real absolute time. */

  nsec = ((nsec * 100) / g_time_ratio) + g_start;

#ifdef __APPLE__
  dispatch_time_t start;

  struct timespec now;
  uint64_t now_ns;

  clock_gettime(CLOCK_MONOTONIC, &now);
  now_ns = 1000000000ull * now.tv_sec + now.tv_nsec;
  start = dispatch_walltime(NULL, nsec - now_ns);
  dispatch_source_set_timer(g_timer, start, DISPATCH_TIME_FOREVER, 1000);
  return 0;
#else
  struct itimerspec tspec =
    {
      0
    };

  tspec.it_value.tv_sec  = nsec / 1000000000;
  tspec.it_value.tv_nsec = nsec % 1000000000;

  return timer_settime(g_timer, TIMER_ABSTIME, &tspec, NULL);
#endif
}

/****************************************************************************
 * Name: host_timerirq
 *
 * Description:
 *   Get timer irq
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, irq num returned, otherwise a negative value.
 *
 ****************************************************************************/

int host_timerirq(void)
{
  return SIGALRM;
}
