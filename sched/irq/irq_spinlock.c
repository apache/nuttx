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
#include "irq/irq.h"

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

void spinlock_mark_locking(FAR spinlock_debug_t *info)
{
  FAR struct tcb_s *rtcb = running_task();

#ifdef CONFIG_SMP
  /* The critical-section lock is allowed to be acquired while another
   * spinlock is being waited for (e.g. from the assert path), so it is
   * not marked to avoid false multi-wait detection.
   */

  if (info == &g_cpu_irqlock.info)
    {
      return;
    }
#endif

  /* Calling `spinlock_mark_locking` does not necessarily disable
   * interrupts. If interrupts are not disabled, an interrupt might
   * corrupt the previous `wait_spinlock` information. So, interrupt
   * context is not considered.
   */

  if (up_interrupt_context())
    {
      return;
    }

  /* Check if the current tcb is not waiting for other spinlock. */

  if (rtcb->wait_spinlock != NULL)
    {
      _alert("Detect thread(%d) is waiting for multiple spinlocks at the "
             "same time, lock1(%p) and lock2 (%p)!\n",
             rtcb->pid, rtcb->wait_spinlock, info);
      dump_spinlock(&rtcb->hold_spinlock);
      PANIC();
    }

  /* Only the method of acquiring lock through busywait needs to be marked.
   * If the lock is acquired through trylock, no marking is required
   * because it will not cause a circular wait and lead to deadlock.
   */

  rtcb->wait_spinlock = info;
}

void spinlock_mark_locked(FAR spinlock_debug_t *info)
{
  FAR struct tcb_s *rtcb = running_task();

#ifdef CONFIG_SMP
  /* The critical-section lock is allowed to be acquired while another
   * spinlock is being waited for (e.g. from the assert path), so it is
   * not marked to avoid false multi-wait detection.
   */

  if (info == &g_cpu_irqlock.info)
    {
      return;
    }
#endif

  if (up_interrupt_context())
    {
      return;
    }

  /* Check if the spinlock has been held. */

  if (info->holder != NULL)
    {
      _alert("Detect thread(%d) relocks spinlock(%p)!\n",
             rtcb->pid, info);
      dump_spinlock(&rtcb->hold_spinlock);
      PANIC();
    }

  /* Check if the current tcb acquires the right lock. */

  if (rtcb->wait_spinlock != info)
    {
      _alert("Detect thread(%d) wants to acquire lock1(%p), but ends up "
             "acquiring lock2(%p)!\n",
             rtcb->pid, rtcb->wait_spinlock, info);
      dump_spinlock(&rtcb->hold_spinlock);
      PANIC();
    }

  /* Set info and enqueue. */

  info->holder = rtcb;
  info->caller = return_address(0);
  sq_addlast(&info->flink, &rtcb->hold_spinlock);

  /* Reset wait_spinlock. */

  rtcb->wait_spinlock = NULL;
}

void spinlock_mark_unlocked(FAR spinlock_debug_t *info)
{
  FAR struct tcb_s *rtcb = running_task();

#ifdef CONFIG_SMP
  /* The critical-section lock is allowed to be acquired while another
   * spinlock is being waited for (e.g. from the assert path), so it is
   * not marked to avoid false multi-wait detection.
   */

  if (info == &g_cpu_irqlock.info)
    {
      return;
    }
#endif

  if (up_interrupt_context())
    {
      return;
    }

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
  if (!sq_empty(&from->hold_spinlock) && !up_interrupt_context())
    {
      _alert("Detect thread(%d) switches context while holding spinlock!\n",
             from->pid);
      dump_spinlock(&from->hold_spinlock);
      PANIC();
    }
}

/* Get the pid of the current lock holder as a reference-free snapshot:
 * take the holder pid and drop the pointer immediately.  Note that this
 * is inherently racy: the holder may release the lock or exit at any
 * time, so the pid is valid only at the moment of the read.
 */

pid_t spinlock_get_holder(FAR spinlock_debug_t *info)
{
  if (info->holder != NULL)
    {
      return info->holder->pid;
    }

  return INVALID_PROCESS_ID;
}

#endif /* CONFIG_SPINLOCK_DEBUG */
