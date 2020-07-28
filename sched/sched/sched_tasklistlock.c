/****************************************************************************
 * sched/sched/sched_tasklistlock.c
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
 * Name: nxsched_lock_tasklist()
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
 *   the interrupts prior to the call to nxsched_lock_tasklist();
 ****************************************************************************/

irqstate_t nxsched_lock_tasklist(void)
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
 * Name: nxsched_unlock_tasklist()
 *
 * Description:
 *   Decrement the call counter (g_tasklist_lock_count[cpu]) and if it
 *   decrements to zero then release the spinlock (g_tasklist_lock) and
 *   restore the interrupt state as it was prior to the previous call to
 *   nxsched_lock_tasklist().
 *
 *   NOTE: This API is used to protect tasklists in the scheduler. So do not
 *   use this API for other purposes.
 *
 * Input Parameters:
 *   lock - The architecture-specific value that represents the state of
 *          the interrupts prior to the call to nxsched_lock_tasklist().
 *
 * Returned Value:
 *   None
 ****************************************************************************/

void nxsched_unlock_tasklist(irqstate_t lock)
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
