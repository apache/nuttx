/****************************************************************************
 * arch/sim/src/sim/up_hosttime.c
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
 *   irq - a pointer where we save the host signal number for SIGALRM
 *
 * Returned Value:
 *   On success, (0) zero value is returned, otherwise a negative value.
 *
 ****************************************************************************/

int host_settimer(int *irq)
{
  struct itimerval it;

  if (irq == NULL)
    {
      return -EINVAL;
    }

  *irq = SIGALRM;

  it.it_interval.tv_sec  = 0;
  it.it_interval.tv_usec = CONFIG_USEC_PER_TICK;
  it.it_value            = it.it_interval;

  /* Start a host timer at a rate indicated by CONFIG_USEC_PER_TICK */

  return setitimer(ITIMER_REAL, &it, NULL);
}
