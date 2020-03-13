/****************************************************************************
 * sched/wdog/wd_initialize.c
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

/* This is wdog tickbase, for wd_gettime() may called many times
 * between 2 times of wd_timer(), we use it to update wd_gettime().
 */

#ifdef CONFIG_SCHED_TICKLESS
clock_t g_wdtickbase;
#endif

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
