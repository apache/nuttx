/****************************************************************************
 * sched/sched/sched_tasklistlock.c
 *
 *   Copyright 2018 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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
#include <nuttx/spinlock.h>

#include <sys/types.h>
#include <arch/irq.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Splinlock to protect the tasklists */

static volatile spinlock_t g_tasklist_lock SP_SECTION = SP_UNLOCKED;

/* Handles nested calls */

static volatile uint8_t g_tasklist_lock_count[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_tasklist_lock()
 *
 * Description:
 *   Disable local interrupts and take the global spinlock (g_tasklist_lock)
 *   if the call counter (g_tasklist_lock_count[cpu]) equals to 0. Then the
 *   counter on the CPU is incremented to allow nested call.
 *
 *   NOTE: This API is used to protect tasklists in the scheduler. So do not
 *   use this API for other purposes.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to sched_tasklist_lock();
 ****************************************************************************/

irqstate_t sched_tasklist_lock(void)
{
  int me;
  irqstate_t ret;

  ret = up_irq_save();
  me  = this_cpu();

  if (0 == g_tasklist_lock_count[me])
    {
      spin_lock(&g_tasklist_lock);
    }

  g_tasklist_lock_count[me]++;
  DEBUGASSERT(0 != g_tasklist_lock_count[me]);
  return ret;
}

/****************************************************************************
 * Name: sched_tasklist_unlock()
 *
 * Description:
 *   Decrement the call counter (g_tasklist_lock_count[cpu]) and if it
 *   decrements to zero then release the spinlock (g_tasklist_lock) and
 *   restore the interrupt state as it was prior to the previous call to
 *   sched_tasklist_lock().
 *
 *   NOTE: This API is used to protect tasklists in the scheduler. So do not
 *   use this API for other purposes.
 *
 * Input Parameters:
 *   lock - The architecture-specific value that represents the state of
 *          the interrupts prior to the call to sched_tasklist_lock().
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void sched_tasklist_unlock(irqstate_t lock)
{
  int me;

  me = this_cpu();

  DEBUGASSERT(0 < g_tasklist_lock_count[me]);
  g_tasklist_lock_count[me]--;

  if (0 == g_tasklist_lock_count[me])
    {
      spin_unlock(&g_tasklist_lock);
    }

  up_irq_restore(lock);
}
