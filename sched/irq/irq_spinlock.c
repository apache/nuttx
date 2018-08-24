/****************************************************************************
 * sched/irq/irq_spinlock.c
 *
 *   Copyright 2017,2018 Sony Video & Sound Products Inc.
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

#if defined(CONFIG_SMP) && defined (CONFIG_SPINLOCK_IRQ) && \
    defined(CONFIG_ARCH_GLOBAL_IRQDISABLE)

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

#endif /* CONFIG_SMP && CONFIG_SPINLOCK_IRQ && CONFIG_ARCH_GLOBAL_IRQDISABLE */
