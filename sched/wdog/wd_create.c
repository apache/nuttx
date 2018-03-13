/****************************************************************************
 * sched/wdog/wd_create.c
 *
 *   Copyright (C) 2007-2009, 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <queue.h>

#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>

#include "wdog/wdog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_create
 *
 * Description:
 *   The wd_create function will create a watchdog timer by allocating one
 *   from the list of free watchdog timers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Pointer to watchdog (i.e., the watchdog ID), or NULL if insufficient
 *   watchdogs are available.
 *
 ****************************************************************************/

WDOG_ID wd_create (void)
{
  FAR struct wdog_s *wdog;
  irqstate_t flags;

  /* These actions must be atomic with respect to other tasks and also with
   * respect to interrupt handlers that may be allocating or freeing watchdog
   * timers.
   */

  flags = enter_critical_section();

  /* If we are in an interrupt handler -OR- if the number of pre-allocated
   * timer structures exceeds the reserve, then take the next timer from
   * the head of the free list.
   */

  if (g_wdnfree > CONFIG_WDOG_INTRESERVE || up_interrupt_context())
    {
      /* Remove the watchdog timer from the free list */

      wdog = (FAR struct wdog_s *)sq_remfirst(&g_wdfreelist);

      /* Did we get one? */

      if (wdog != NULL)
        {
          /* Yes.. decrement the count of free, pre-allocated timers (all
           * with interrupts disabled).
           */

          DEBUGASSERT(g_wdnfree > 0);
          g_wdnfree--;

          /* Clear the forward link and all flags */

          wdog->next  = NULL;
          wdog->flags = 0;
        }
      else
        {
          /* We didn't get one... The count should then be exactly zero */

          DEBUGASSERT(g_wdnfree == 0);
        }

      leave_critical_section(flags);
    }

  /* We are in a normal tasking context AND there are not enough unreserved,
   * pre-allocated watchdog timers.  We need to allocate one from the kernel
   * heap.
   */

  else
    {
      /* We do not require that interrupts be disabled to do this. */

      leave_critical_section(flags);
      wdog = (FAR struct wdog_s *)kmm_malloc(sizeof(struct wdog_s));

      /* Did we get one? */

      if (wdog)
        {
          /* Yes.. Clear the forward link and set the allocated flag */

          wdog->next  = NULL;
          wdog->flags = WDOGF_ALLOCED;
        }
    }

  return (WDOG_ID)wdog;
}
