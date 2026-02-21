/****************************************************************************
 * arch/sim/src/sim/posix/sim_hosttime.c
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

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "sim_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Start time of the simulation in nanoseconds (monotonic clock) */

static uint64_t g_start;

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

  return 0;
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
 ****************************************************************************/

int host_settimer(uint64_t nsec)
{
  struct itimerval it;
  uint64_t now;
  uint64_t usec;

  now = host_gettime(false);

  usec = (nsec <= now) ? 1 : (nsec - now) / 1000;
  usec = (usec == 0) ? 1 : usec;

  it.it_value.tv_sec  = usec / 1000000;
  it.it_value.tv_usec = usec % 1000000;

  /* One-shot timer */

  it.it_interval.tv_sec  = 0;
  it.it_interval.tv_usec = 0;

  return setitimer(ITIMER_REAL, &it, NULL);
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
