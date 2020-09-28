/****************************************************************************
 * arch/sim/src/sim/up_hosttime.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
  struct timespec tp;

  clock_gettime(rtc ? CLOCK_REALTIME : CLOCK_MONOTONIC, &tp);
  return 1000000000ull * tp.tv_sec + tp.tv_nsec;
}

/****************************************************************************
 * Name: host_sleep
 ****************************************************************************/

void host_sleep(uint64_t nsec)
{
  usleep(nsec);
}

/****************************************************************************
 * Name: host_sleepuntil
 ****************************************************************************/

void host_sleepuntil(uint64_t nsec)
{
  static uint64_t base;
  uint64_t now;

  now = host_gettime(false);
  if (base == 0)
    {
      base = now;
    }

  now -= base;

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
