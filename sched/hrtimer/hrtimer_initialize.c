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

#include <hrtimer/hrtimer.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Spinlock protecting the high-resolution timer RB-tree */

spinlock_t g_hrtspinlock = SP_UNLOCKED;

/* Red-Black tree containing all active high-resolution timers */

struct hrtimer_tree_s g_activetree = RB_INITIALIZER(g_activetree);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: RB_GENERATE
 *
 * Description:
 *   Generate the Red-Black tree implementation for high-resolution timers.
 *   This macro instantiates all necessary functions for managing the
 *   RB-tree including insert, remove, and search operations.
 *
 * Assumptions/Notes:
 *   - The RB-tree is keyed by the expiration time of hrtimer_node_s.
 *   - Used internally by hrtimer_start, hrtimer_process, and hrtimer_cancel.
 ****************************************************************************/

RB_GENERATE(hrtimer_tree_s, hrtimer_node_s, entry, hrtimer_cmp);
