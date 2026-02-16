/****************************************************************************
 * sched/hrtimer/hrtimer_initialize.c
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

#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Array of pointers to currently running high-resolution timers
 * for each CPU in SMP configurations. Index corresponds to CPU ID.
 */

#ifdef CONFIG_SMP
uintptr_t g_hrtimer_running[CONFIG_SMP_NCPUS];
#endif

/* Global spinlock protecting the high-resolution timer subsystem.
 *
 * This spinlock serializes access to the hrtimer queue and
 * protects timer state transitions. It must be held whenever the
 * timer queue is modified.
 */

seqcount_t g_hrtimer_lock = SEQLOCK_INITIALIZER;

/* HRTimer queue for all active high-resolution timers.
 *
 * When CONFIG_HRTIMER_TREE is enabled, timers are stored in a queue.
 * When disabled, timers are stored in a linked list.
 *
 * The queue is ordered by absolute expiration time in
 * both configurations.
 */

struct hrtimer_s g_hrtimer_guard =
{
#ifdef CONFIG_HRTIMER_TREE
  { NULL },
#else
  { &g_hrtimer_list, &g_hrtimer_list },
#endif
  NULL,
  INT64_MAX
};

#ifdef CONFIG_HRTIMER_TREE
struct hrtimer_tree_s g_hrtimer_tree =
{
  &g_hrtimer_guard
};
struct FAR hrtimer_s *g_hrtimer_head = &g_hrtimer_guard;
#else
struct list_node g_hrtimer_list =
{
  &g_hrtimer_guard.node, &g_hrtimer_guard.node
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: RB_GENERATE
 *
 * Description:
 *   Instantiate the queue helper functions for the hrtimer
 *   subsystem.
 *
 *   This macro generates the static inline functions required to
 *   manipulate the hrtimer queue, including insertion,
 *   removal, and lookup operations.
 *
 *   Note: This is only compiled when CONFIG_HRTIMER_TREE is enabled.
 *
 * Assumptions/Notes:
 *   - The tree key is the absolute expiration time stored in
 *     hrtimer_node_s and compared via hrtimer_compare().
 *   - All accesses to the tree must be serialized using
 *     g_hrtimer_lock.
 *   - These generated functions are used internally by the hrtimer
 *     core (e.g., hrtimer_start(), hrtimer_cancel(), and expire paths).
 ****************************************************************************/

#ifdef CONFIG_HRTIMER_TREE
RB_GENERATE(hrtimer_tree_s, hrtimer_s, node, hrtimer_compare);
#endif
