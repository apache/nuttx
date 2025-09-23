/****************************************************************************
 * sched/irq/irq_spinlock.c
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

/* When the inline spinlock helpers are disabled, emit callable non-inline
 * versions of the helpers defined in <nuttx/spinlock.h> from this file.
 */

#ifndef CONFIG_SPINLOCK_INLINE
#  define CONFIG_SPINLOCK_INLINE
#  define SPINLOCK_INLINE
#endif

#include <nuttx/spinlock.h>

#include <assert.h>
#include <debug.h>
#include <sys/types.h>
#include <arch/irq.h>

#include "sched/sched.h"

#if defined(CONFIG_SPINLOCK)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK_DEBUG
static void dump_spinlock(FAR sq_queue_t *hold_spinlock)
{
  FAR sq_entry_t *entry;

  sq_for_every(hold_spinlock, entry)
    {
      FAR spinlock_debug_t *info = (FAR spinlock_debug_t *)entry;

      _alert("Spinlock(%p) has been locked by %p\n", info, info->caller);
    }
}

void spinlock_mark_locked(FAR spinlock_debug_t *info)
{
  FAR struct tcb_s *rtcb = running_task();

  /* Check if the spinlock has been held. */

  if (info->holder != NULL)
    {
      _alert("Detect thread(%d) relocks spinlock(%p)!\n",
             rtcb->pid, info);
      dump_spinlock(&rtcb->hold_spinlock);
      PANIC();
    }

  /* Set info and enqueue. */

  info->holder = rtcb;
  info->caller = return_address(0);
  sq_addlast(&info->flink, &rtcb->hold_spinlock);
}

void spinlock_mark_unlocked(FAR spinlock_debug_t *info)
{
  FAR struct tcb_s *rtcb = running_task();

  /* Check if locking and unlocking are the same task. */

  if (info->holder != rtcb)
    {
      _alert("Detect thread(%d) unlocks spinlock(%p) which is not held!\n",
             rtcb->pid, info);
      dump_spinlock(&rtcb->hold_spinlock);
      PANIC();
    }

  /* Currently, NuttX does not support unlock in the reverse order, since
   * this will pollute flags. So check if unlocking is the reverse order.
   */

  if (sq_tail(&rtcb->hold_spinlock) != &info->flink)
    {
      _alert("Detect thread(%d) unlocks spinlock(%p) in the wrong order!\n",
             rtcb->pid, info);
      dump_spinlock(&rtcb->hold_spinlock);
      PANIC();
    }

  /* Reset info and dequeue. */

  info->holder = NULL;
  info->caller = NULL;
  sq_remlast(&rtcb->hold_spinlock);
}

void spinlock_switch_context(FAR struct tcb_s *from)
{
  if (!sq_empty(&from->hold_spinlock))
    {
      _alert("Detect thread(%d) switches context while holding spinlock!\n",
             from->pid);
      dump_spinlock(&from->hold_spinlock);
      PANIC();
    }
}

#endif /* CONFIG_SPINLOCK_DEBUG */

#endif /* CONFIG_SPINLOCK */
