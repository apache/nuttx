/****************************************************************************
 * sched/wdog/wd_cancel.c
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>

#include "sched/sched.h"
#include "wdog/wdog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_cancel
 *
 * Description:
 *   This function cancels a currently running watchdog timer. Watchdog
 *   timers may be canceled from the interrupt level.
 *
 * Input Parameters:
 *   wdog - ID of the watchdog to cancel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success;  A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int wd_cancel(FAR struct wdog_s *wdog)
{
  FAR struct wdog_s *curr;
  FAR struct wdog_s *prev;
  irqstate_t flags;
  int ret = -EINVAL;

  /* Prohibit timer interactions with the timer queue until the
   * cancellation is complete
   */

  flags = enter_critical_section();

  /* Make sure that the watchdog is initialized (non-NULL) and is still
   * active.
   */

  if (wdog != NULL && WDOG_ISACTIVE(wdog))
    {
      /* Search the g_wdactivelist for the target FCB.  We can't use sq_rem
       * to do this because there are additional operations that need to be
       * done.
       */

      prev = NULL;
      curr = (FAR struct wdog_s *)g_wdactivelist.head;

      while ((curr) && (curr != wdog))
        {
          prev = curr;
          curr = curr->next;
        }

      /* Check if the watchdog was found in the list.  If not, then an OS
       * error has occurred because the watchdog is marked active!
       */

      DEBUGASSERT(curr);

      /* If there is a watchdog in the timer queue after the one that
       * is being canceled, then it inherits the remaining ticks.
       */

      if (curr->next)
        {
          curr->next->lag += curr->lag;
        }

      /* Now, remove the watchdog from the timer queue */

      if (prev)
        {
          /* Remove the watchdog from mid- or end-of-queue */

          sq_remafter((FAR sq_entry_t *)prev, &g_wdactivelist);
        }
      else
        {
          /* Remove the watchdog at the head of the queue */

          sq_remfirst(&g_wdactivelist);

          /* Reassess the interval timer that will generate the next
           * interval event.
           */

          nxsched_reassess_timer();
        }

      /* Mark the watchdog inactive */

      wdog->next = NULL;
      WDOG_CLRACTIVE(wdog);

      /* Return success */

      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}
