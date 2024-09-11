/****************************************************************************
 * sched/wdog/wd_cancel.c
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
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  ret = wd_cancel_irq(wdog);

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: wd_cancel_irq
 *
 * Description:
 *   This function cancels a currently running watchdog timer. Watchdog
 *   timers may be cancelled from the interrupt level.  This function is
 *   intended to be called from critical sections.
 *
 * Input Parameters:
 *   wdog - ID of the watchdog to cancel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success;  A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int wd_cancel_irq(FAR struct wdog_s *wdog)
{
  if (wdog == NULL)
    {
      return -EINVAL;
    }

  /* Prohibit timer interactions with the timer queue until the
   * cancellation is complete
   */

  /* Make sure that the watchdog is still active. */

  if (WDOG_ISACTIVE(wdog))
    {
      bool head = list_is_head(&g_wdactivelist, &wdog->node);

      /* Now, remove the watchdog from the timer queue */

      list_delete(&wdog->node);

      /* Mark the watchdog inactive */

      wdog->func = NULL;

      if (head)
        {
          /* If the watchdog is at the head of the timer queue, then
           * we will need to re-adjust the interval timer that will
           * generate the next interval event.
           */

          nxsched_reassess_timer();
        }
    }

  return OK;
}
