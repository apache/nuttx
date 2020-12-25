/****************************************************************************
 * sched/irq/irq_spinlock.c
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

#if defined(CONFIG_SMP) && defined (CONFIG_SPINLOCK_IRQ)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Used for access control */

static volatile spinlock_t g_irq_spin SP_SECTION = SP_UNLOCKED;

/* Handles nested calls to spin_lock_irqsave and spin_unlock_irqrestore */

static volatile uint8_t g_irq_spin_count[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spin_lock_irqsave
 *
 * Description:
 *   If SMP and SPINLOCK_IRQ are enabled:
 *     Disable local interrupts and take the global spinlock (g_irq_spin)
 *     if the call counter (g_irq_spin_count[cpu]) equals to 0. Then the
 *     counter on the CPU is increment to allow nested call.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. But do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP and SPINLOCK_IRQ are not enabled:
 *     This function is equivalent to enter_critical_section().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to spin_lock_irqsave();
 *
 ****************************************************************************/

irqstate_t spin_lock_irqsave(void)
{
  irqstate_t ret;
  ret = up_irq_save();

  int me = this_cpu();
  if (0 == g_irq_spin_count[me])
    {
      spin_lock(&g_irq_spin);
    }

  g_irq_spin_count[me]++;
  DEBUGASSERT(0 != g_irq_spin_count[me]);
  return ret;
}

/****************************************************************************
 * Name: spin_unlock_irqrestore
 *
 * Description:
 *   If SMP and SPINLOCK_IRQ are enabled:
 *     Decrement the call counter (g_irq_spin_count[cpu]) and if it
 *     decrements to zero then release the spinlock (g_irq_spin) and
 *     restore the interrupt state as it was prior to the previous call to
 *     spin_lock_irqsave().
 *
 *   If SMP and SPINLOCK_IRQ are not enabled:
 *     This function is equivalent to leave_critical_section().
 *
 * Input Parameters:
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to spin_lock_irqsave();
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spin_unlock_irqrestore(irqstate_t flags)
{
  int me = this_cpu();

  DEBUGASSERT(0 < g_irq_spin_count[me]);
  g_irq_spin_count[me]--;

  if (0 == g_irq_spin_count[me])
    {
      spin_unlock(&g_irq_spin);
    }

  up_irq_restore(flags);
}

#endif /* CONFIG_SMP && CONFIG_SPINLOCK_IRQ */
