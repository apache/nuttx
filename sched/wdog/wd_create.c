/****************************************************************************
 * sched/wdog/wd_create.c
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
