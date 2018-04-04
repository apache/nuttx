/****************************************************************************
 * sched/sched/sched_thistask.c
 *
 *   Copyright 2018 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *           Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <arch/irq.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "sched/sched.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: this_task
 *
 * Description:
 *   The functions will safely obtain the TCB that is currently running
 *   on the current CPU. In SMP, this must be done by disabling local
 *   interrupts to avoid CPU switching during access to current_task()
 *
 * Returned Value:
 *   the TCB that is currently running on the current CPU.
 *
 ****************************************************************************/

FAR struct tcb_s *this_task(void)
{
  FAR struct tcb_s *tcb;
#if defined(CONFIG_ARCH_GLOBAL_IRQDISABLE)
  irqstate_t flags;

  /* If the CPU supports suppression of interprocessor interrupts, then simple
   * disabling interrupts will provide sufficient protection for the following
   * operations.
   */

  flags = up_irq_save();
#elif defined(CONFIG_ARCH_HAVE_FETCHADD)
  /* Global locking is supported and, hence, sched_lock() will provide the
   * necessary protection.
   */

  sched_lock();
#else
  /* REVISIT:  Otherwise, there is no protection available.  sched_lock() and
   * enter_critical section are not viable options here (because both depend
   * on this_task()).
   */
#  warning "Missing critical section"
#endif

  /* Obtain the TCB which is currently running on this CPU */

  tcb = current_task(this_cpu());

  /* Enable local interrupts */

#if defined(CONFIG_ARCH_GLOBAL_IRQDISABLE)
  up_irq_restore(flags);
#elif defined(CONFIG_ARCH_HAVE_FETCHADD)
  sched_unlock();
#endif
  return tcb;
}

#endif /* CONFIG_SMP */
