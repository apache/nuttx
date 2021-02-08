/****************************************************************************
 * sched/signal/sig_releasependingsigaction.c
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

#include <sched.h>

#include <nuttx/irq.h>

#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_release_pendingsigaction
 *
 * Description:
 *   Deallocate a pending signal action Q entry
 *
 ****************************************************************************/

void nxsig_release_pendingsigaction(FAR sigq_t *sigq)
{
  irqstate_t flags;

  /* If this is a generally available pre-allocated structure,
   * then just put it back in the free list.
   */

  if (sigq->type == SIG_ALLOC_FIXED)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)sigq, &g_sigpendingaction);
      leave_critical_section(flags);
    }

  /* If this is a message pre-allocated for interrupts,
   * then put it back in the correct  free list.
   */

  else if (sigq->type == SIG_ALLOC_IRQ)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)sigq, &g_sigpendingirqaction);
      leave_critical_section(flags);
    }

  /* Otherwise, deallocate it.  Note:  interrupt handlers
   * will never deallocate signals because they will not
   * receive them.
   */

  else if (sigq->type == SIG_ALLOC_DYN)
    {
      kmm_free(sigq);
    }
}
