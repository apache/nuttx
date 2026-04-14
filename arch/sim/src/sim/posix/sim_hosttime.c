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

  return rtc ? current : current - g_start;
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
      usleep((nsec - now) / 1000);
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
  /* Convert to microseconds and set minimum timer to 1 microsecond. */

  nsec += g_start;

#ifdef __APPLE__
  dispatch_time_t start;

  start = dispatch_walltime(NULL, nsec - host_gettime(true));
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
