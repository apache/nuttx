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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_gettime
 ****************************************************************************/

uint64_t host_gettime(bool rtc)
{
  static uint64_t start;
  struct timespec tp;
  uint64_t current;

  clock_gettime(rtc ? CLOCK_REALTIME : CLOCK_MONOTONIC, &tp);
  current = 1000000000ull * tp.tv_sec + tp.tv_nsec;

  if (rtc)
    {
      return current;
    }

  if (start == 0)
    {
      start = current;
    }

  return current - start;
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
  struct itimerval it;

  it.it_interval.tv_sec  = 0;
  it.it_interval.tv_usec = 0;
  it.it_value.tv_sec     = nsec / 1000000000;
  it.it_value.tv_usec    = (nsec + 1000) / 1000;

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
