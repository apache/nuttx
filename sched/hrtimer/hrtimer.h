/****************************************************************************
 * sched/hrtimer/hrtimer.h
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

#ifndef __SCHED_HRTIMER_HRTIMER_H
#define __SCHED_HRTIMER_HRTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/hrtimer.h>

#ifdef CONFIG_HRTIMER

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Red-black tree head for managing active high-resolution timers.
 *
 * Timers are ordered by expiration time, the earliest expiring timer
 * being the left-most (minimum) node in the tree.
 */

RB_HEAD(hrtimer_tree_s, hrtimer_node_s);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Spinlock protecting access to the hrtimer RB-tree and timer state */

extern spinlock_t g_hrtimer_spinlock;

/* Red-Black tree containing all active high-resolution timers */

extern struct hrtimer_tree_s g_hrtimer_tree;

/* Array of pointers to currently running high-resolution timers
 * for each CPU in SMP configurations. Index corresponds to CPU ID.
 */

#ifdef CONFIG_SMP
extern FAR hrtimer_t *g_hrtimer_running[CONFIG_SMP_NCPUS];
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_process
 *
 * Description:
 *   Called from the timer interrupt handler to process expired
 *   high-resolution timers. If a timer has expired, its callback
 *   function will be executed in the context of the timer interrupt.
 *
 * Input Parameters:
 *   now - The current time (nsecs).
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void hrtimer_process(uint64_t now);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_gettime
 *
 * Description:
 *   Get the current high-resolution time in nanoseconds.
 *
 * Returned Value:
 *   Current time in nanoseconds.
 ****************************************************************************/

static inline_function
uint64_t hrtimer_gettime(void)
{
  struct timespec ts;

  /* Get current time from platform-specific timer */

  clock_systime_timespec(&ts);

  /* Convert timespec to nanoseconds */

  return clock_time2nsec(&ts);
}

/****************************************************************************
 * Name: hrtimer_starttimer
 *
 * Description:
 *   Start the hardware timer to expire at a specified nanosecond time.
 *   Converts the nanosecond time to timespec and calls the platform-specific
 *   timer start function.
 *
 * Input Parameters:
 *   ns - Expiration time in nanoseconds.
 *
 * Returned Value:
 *   OK (0) on success, negated errno on failure.
 ****************************************************************************/

static inline_function
int hrtimer_starttimer(uint64_t ns)
{
  struct timespec ts;
  int ret;

  /* Convert nanoseconds to timespec */

  clock_nsec2time(&ts, ns);

#ifdef CONFIG_ALARM_ARCH
  ret = up_alarm_start(&ts);
#elif defined(CONFIG_TIMER_ARCH)
  ret = up_timer_start(&ts);
#endif

  return ret;
}

/****************************************************************************
 * Name: hrtimer_compare
 *
 * Description:
 *   Compare two high-resolution timer nodes to determine their ordering
 *   in the red-black tree. Used internally by the RB-tree macros.
 *
 * Input Parameters:
 *   a - Pointer to the first hrtimer node.
 *   b - Pointer to the second hrtimer node.
 *
 * Returned Value:
 *   <0 if a expires before b
 *   >=0 if a expires after b
 ****************************************************************************/

static inline_function
int hrtimer_compare(FAR const hrtimer_node_t *a,
                    FAR const hrtimer_node_t *b)
{
  FAR const hrtimer_t *atimer = (FAR const hrtimer_t *)a;
  FAR const hrtimer_t *btimer = (FAR const hrtimer_t *)b;

  return clock_compare(atimer->expired, btimer->expired) ? -1 : 1;
}

/****************************************************************************
 * Red-Black Tree Prototype for high-resolution timers
 *
 * Description:
 *   Declare the RB-tree prototype that manages all active high-resolution
 *   timers. This tree provides efficient insertion, removal, and lookup
 *   operations based on timer expiration time.
 ****************************************************************************/

RB_PROTOTYPE(hrtimer_tree_s, hrtimer_node_s, entry, hrtimer_compare);

/****************************************************************************
 * Name: hrtimer_is_armed
 *
 * Description:
 *   Test whether a timer is currently armed (inserted into the RB-tree).
 *
 * Returned Value:
 *   true if armed, false otherwise.
 ****************************************************************************/

static inline_function bool hrtimer_is_armed(FAR hrtimer_t *hrtimer)
{
  return hrtimer->func != NULL;
}

/****************************************************************************
 * Name: hrtimer_remove
 *
 * Description:
 *   Remove a timer from the RB-tree and mark it as unarmed.
 ****************************************************************************/

static inline_function void hrtimer_remove(FAR hrtimer_t *hrtimer)
{
  RB_REMOVE(hrtimer_tree_s, &g_hrtimer_tree, &hrtimer->node);

  /* Explicitly clear parent to mark the timer as unarmed */

  hrtimer->func = NULL;
}

/****************************************************************************
 * Name: hrtimer_insert
 *
 * Description:
 *   Insert a timer into the RB-tree according to its expiration time.
 ****************************************************************************/

static inline_function void hrtimer_insert(FAR hrtimer_t *hrtimer)
{
  RB_INSERT(hrtimer_tree_s, &g_hrtimer_tree, &hrtimer->node);
}

/****************************************************************************
 * Name: hrtimer_get_first
 *
 * Description:
 *   Return the earliest expiring armed timer.
 *
 * Returned Value:
 *   Pointer to the earliest timer, or NULL if none are armed.
 ****************************************************************************/

static inline_function FAR hrtimer_t *hrtimer_get_first(void)
{
  return (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_hrtimer_tree);
}

/****************************************************************************
 * Name: hrtimer_is_first
 *
 * Description:
 *   Test whether the given high-resolution timer is the earliest
 *   expiring timer in the RB-tree.
 *
 *   In a red-black tree ordered by expiration time, the earliest timer
 *   is represented by the left-most node. Therefore, a timer is the
 *   earliest one if it has no left child.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer to be tested.
 *
 * Returned Value:
 *   true  - The timer is the earliest expiring armed timer.
 *   false - The timer is not the earliest timer.
 ****************************************************************************/

static inline_function bool hrtimer_is_first(FAR hrtimer_t *hrtimer)
{
  return RB_LEFT(&hrtimer->node, entry) == NULL;
}

#endif /* CONFIG_HRTIMER */
#endif /* __SCHED_HRTIMER_HRTIMER_H */
