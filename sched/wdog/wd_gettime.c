/****************************************************************************
 * sched/wdog/wd_gettime.c
 *
 *   Copyright (C) 2007, 2009, 2014-2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/wdog.h>
#include <nuttx/irq.h>

#include "wdog/wdog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_gettime
 *
 * Description:
 *   This function returns the time remaining before the specified watchdog
 *   timer expires.
 *
 * Input Parameters:
 *   wdog - watchdog ID
 *
 * Returned Value:
 *   The time in system ticks remaining until the watchdog time expires.
 *   Zero means either that wdog is not valid or that the wdog has already
 *   expired.
 *
 ****************************************************************************/

int wd_gettime(WDOG_ID wdog)
{
  irqstate_t flags;

  /* Verify the wdog */

  flags = enter_critical_section();
  if (wdog != NULL && WDOG_ISACTIVE(wdog))
    {
      /* Traverse the watchdog list accumulating lag times until we find the
       * wdog that we are looking for
       */

      FAR struct wdog_s *curr;
      int delay = 0;

      for (curr = (FAR struct wdog_s *)g_wdactivelist.head;
           curr != NULL;
           curr = curr->next)
        {
          delay += curr->lag;
          if (curr == wdog)
            {
              leave_critical_section(flags);
              return delay;
            }
        }
    }

  leave_critical_section(flags);
  return 0;
}
