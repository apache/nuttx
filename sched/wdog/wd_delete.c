/****************************************************************************
 * sched/wdog/wd_delete.c
 *
 *   Copyright (C) 2007-2009, 2014, 2016, 2018 Gregory Nutt. All rights
 *     reserved.
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

  DEBUGASSERT(wdog);

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
