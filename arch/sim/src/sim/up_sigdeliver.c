/****************************************************************************
 * arch/sim/src/sim/up_sigdeliver.c
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

#include <stdint.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void sim_sigdeliver(void)
{
  struct tcb_s *rtcb = current_task(this_cpu());

  if (NULL == (rtcb->xcp.sigdeliver))
    {
      return;
    }

  /* NOTE: we enter critical section here for sim instead of
   * up_irq_enable() up_irq_save() used in other architectures
   */

#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = get_errno();

#ifdef CONFIG_SMP
  /* In the SMP case, we must terminate the critical section while the signal
   * handler executes, but we also need to restore the irqcount when the
   * we resume the main thread of the task.
   */

  int16_t saved_irqcount;
#endif

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

  /* NOTE: we do not save the return state for sim */

#ifdef CONFIG_SMP
  /* In the SMP case, up_schedule_sigaction(0) will have incremented
   * 'irqcount' in order to force us into a critical section.  Save the
   * pre-incremented irqcount.
   */

  saved_irqcount = rtcb->irqcount;
  DEBUGASSERT(saved_irqcount >= 0);

  /* Now we need call leave_critical_section() repeatedly to get the irqcount
   * to zero, freeing all global spinlocks that enforce the critical section.
   */

  do
    {
      leave_critical_section(flags);
    }
  while (rtcb->irqcount > 0);
#endif /* CONFIG_SMP */

  /* Deliver the signal */

  ((sig_deliver_t)rtcb->xcp.sigdeliver)(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   *
   * I would prefer that all interrupts are disabled when
   * up_fullcontextrestore() is called, but that may not be necessary.
   */

  sinfo("Resuming\n");

#ifdef CONFIG_SMP
  /* Restore the saved 'irqcount' and recover the critical section
   * spinlocks.
   */

  DEBUGASSERT(rtcb->irqcount == 0);
  while (rtcb->irqcount < saved_irqcount)
    {
      enter_critical_section();
    }
#endif

  /* Restore the saved errno value */

  set_errno(saved_errno);

  /* Allows next handler to be scheduled */

  rtcb->xcp.sigdeliver = NULL;

  /* NOTE: we leave a critical section here for sim */

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif

  /* NOTE: we do not restore the return state for sim */
}
