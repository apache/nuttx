/****************************************************************************
 * sched/wdog/wd_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2014 Gregory Nutt. All rights reserved.
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

#include "wdog/wdog.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The g_wdfreelist data structure is a singly linked list of watchdogs
 * available to the system for delayed function use.
 */

sq_queue_t g_wdfreelist;

/* The g_wdactivelist data structure is a singly linked list ordered by
 * watchdog expiration time. When watchdog timers expire,the functions on
 * this linked list are removed and the function is called.
 */

sq_queue_t g_wdactivelist;

/* This is the number of free, pre-allocated watchdog structures in the
 * g_wdfreelist.  This value is used to enforce a reserve for interrupt
 * handlers.
 */

uint16_t g_wdnfree;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* g_wdpool is a list of pre-allocated watchdogs. The number of watchdogs
 * in the pool is a configuration item.
 */

static struct wdog_s g_wdpool[CONFIG_PREALLOC_WDOGS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_initialize
 *
 * Description:
 * This function initializes the watchdog data structures
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function must be called early in the initialization sequence
 *   before the timer interrupt is attached and before any watchdog
 *   services are used.
 *
 ****************************************************************************/

void wd_initialize(void)
{
  FAR struct wdog_s *wdog = g_wdpool;
  int i;

  /* Initialize watchdog lists */

  sq_init(&g_wdfreelist);
  sq_init(&g_wdactivelist);

  /* The g_wdfreelist must be loaded at initialization time to hold the
   * configured number of watchdogs.
   */

  for (i = 0; i < CONFIG_PREALLOC_WDOGS; i++)
    {
      sq_addlast((FAR sq_entry_t *)wdog++, &g_wdfreelist);
    }

  /* All watchdogs are free */

  g_wdnfree = CONFIG_PREALLOC_WDOGS;
}
