/****************************************************************************
 * sched/wdog/wd_delete.c
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

#include <queue.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>

#include "wdog/wdog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_delete
 *
 * Description:
 *   The wd_delete() function will deallocate a watchdog timer by returning
 *   it to the free pool of watchdog timers.  The watchdog timer will be
 *   removed from the active timer queue if had been started.
 *
 * Input Parameters:
 *   wdog - The watchdog ID to delete.  This is actually a pointer to a
 *          watchdog structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return to
 *   indicate the nature of any failure.
 *
 * Assumptions:
 *   The caller has assured that the watchdog is no longer in use.
 *
 ****************************************************************************/

int wd_delete(WDOG_ID wdog)
{
  irqstate_t flags;

  DEBUGASSERT(wdog != NULL);

  /* The following steps are atomic... the watchdog must not be active when
   * it is being deallocated.
   */

  flags = enter_critical_section();

  /* Check if the watchdog has been started. */

  if (WDOG_ISACTIVE(wdog))
    {
      /* Yes.. stop it */

      wd_cancel(wdog);
    }

  /* Did this watchdog come from the pool of pre-allocated timers?  Or, was
   * it allocated from the heap?
   */

  if (WDOG_ISALLOCED(wdog))
    {
      /* It was allocated from the heap.  Use sched_kfree() to release the
       * memory.  If the timer was released from an interrupt handler,
       * sched_kfree() will defer the actual deallocation of the memory
       * until a more appropriate time.
       *
       * We don't need interrupts disabled to do this.
       */

      leave_critical_section(flags);
      sched_kfree(wdog);
    }

  /* Check if this is pre-allocated timer. */

  else if (!WDOG_ISSTATIC(wdog))
    {
      /* Put the timer back on the free list and increment the count of free
       * timers, all with interrupts disabled.
       */

      sq_addlast((FAR sq_entry_t *)wdog, &g_wdfreelist);
      g_wdnfree++;
      DEBUGASSERT(g_wdnfree <= CONFIG_PREALLOC_WDOGS);
      leave_critical_section(flags);
    }

  /* This function should not be called for statically allocated timers. */

  else
    {
      leave_critical_section(flags);
    }

  /* Return success */

  return OK;
}
