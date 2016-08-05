/****************************************************************************
 * sched/clock/clock_settime.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2014 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "clock/clock.h"
#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_settime
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 ****************************************************************************/

int clock_settime(clockid_t clock_id, FAR const struct timespec *tp)
{
  struct timespec bias;
  irqstate_t flags;
  int ret = OK;

  sinfo("clock_id=%d\n", clock_id);
  DEBUGASSERT(tp != NULL);

  /* CLOCK_REALTIME - POSIX demands this to be present. This is the wall
   * time clock.
   */

  if (clock_id == CLOCK_REALTIME)
    {
#ifndef CONFIG_CLOCK_TIMEKEEPING
      /* Interrupts are disabled here so that the in-memory time
       * representation and the RTC setting will be as close as
       * possible.
       */

      flags = enter_critical_section();

      /* Get the elapsed time since power up (in milliseconds).  This is a
       * bias value that we need to use to correct the base time.
       */

      (void)clock_systimespec(&bias);

      /* Save the new base time. */

      g_basetime.tv_sec  = tp->tv_sec;
      g_basetime.tv_nsec = tp->tv_nsec;

      /* Subtract that bias from the basetime so that when the system
       * timer is again added to the base time, the result is the current
       * time relative to basetime.
       */

      if (g_basetime.tv_nsec < bias.tv_nsec)
        {
          g_basetime.tv_nsec += NSEC_PER_SEC;
          g_basetime.tv_sec--;
        }

      /* Result could be negative seconds */

      g_basetime.tv_nsec -= bias.tv_nsec;
      g_basetime.tv_sec  -= bias.tv_sec;

      /* Setup the RTC (lo- or high-res) */

#ifdef CONFIG_RTC
      if (g_rtc_enabled)
        {
          up_rtc_settime(tp);
        }
#endif
      leave_critical_section(flags);

      sinfo("basetime=(%ld,%lu) bias=(%ld,%lu)\n",
            (long)g_basetime.tv_sec, (unsigned long)g_basetime.tv_nsec,
            (long)bias.tv_sec, (unsigned long)bias.tv_nsec);
#else
      ret = clock_timekeeping_set_wall_time(tp);
#endif
    }
  else
    {
      serr("Returning ERROR\n");
      set_errno(EINVAL);
      ret = ERROR;
    }

  return ret;
}
