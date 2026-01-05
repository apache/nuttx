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

FAR hrtimer_t *g_hrtimer_running[CONFIG_SMP_NCPUS];

/* Global spinlock protecting the high-resolution timer subsystem.
 *
 * This spinlock serializes access to the hrtimer red-black tree and
 * protects timer state transitions. It must be held whenever the
 * timer tree or hrtimer state is modified.
 */

spinlock_t g_hrtimer_spinlock = SP_UNLOCKED;

/* Red-black tree containing all active high-resolution timers.
 *
 * Only timers in the ARMED state are present in this tree. Timers in
 * the RUNNING, CANCELED, or INACTIVE states must not be inserted.
 *
 * The tree is ordered by absolute expiration time.
 */

struct hrtimer_tree_s g_hrtimer_tree = RB_INITIALIZER(g_hrtimer_tree);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: RB_GENERATE
 *
 * Description:
 *   Instantiate the red-black tree helper functions for the hrtimer
 *   subsystem.
 *
 *   This macro generates the static inline functions required to
 *   manipulate the hrtimer red-black tree, including insertion,
 *   removal, and lookup operations.
 *
 * Assumptions/Notes:
 *   - The tree key is the absolute expiration time stored in
 *     hrtimer_node_s and compared via hrtimer_compare().
 *   - All accesses to the tree must be serialized using
 *     g_hrtimer_spinlock.
 *   - These generated functions are used internally by the hrtimer
 *     core (e.g., hrtimer_start(), hrtimer_cancel(), and expire paths).
 ****************************************************************************/

RB_GENERATE(hrtimer_tree_s, hrtimer_node_s, entry, hrtimer_compare);
