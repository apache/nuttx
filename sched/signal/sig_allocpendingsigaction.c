/****************************************************************************
 * sched/signal/sig_allocpendingsigaction.c
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

#include <signal.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_alloc_pendingsigaction
 *
 * Description:
 *   Allocate a new element for the pending signal action queue
 *
 ****************************************************************************/

FAR sigq_t *nxsig_alloc_pendingsigaction(void)
{
  FAR sigq_t    *sigq;
  irqstate_t flags;

  /* Check if we were called from an interrupt handler. */

  if (up_interrupt_context())
    {
      /* Try to get the pending signal action structure from the free list */

      sigq = (FAR sigq_t *)sq_remfirst(&g_sigpendingaction);

      /* If so, then try the special list of structures reserved for
       * interrupt handlers
       */

      if (!sigq)
        {
          sigq = (FAR sigq_t *)sq_remfirst(&g_sigpendingirqaction);
        }
    }

  /* If we were not called from an interrupt handler, then we are
   * free to allocate pending signal action structures if necessary.
   */

  else
    {
      /* Try to get the pending signal action structure from the free list */

      flags = enter_critical_section();
      sigq = (FAR sigq_t *)sq_remfirst(&g_sigpendingaction);
      leave_critical_section(flags);

      /* Check if we got one. */

      if (!sigq)
        {
          /* No...Try the resource pool */

          sigq = (FAR sigq_t *)kmm_malloc((sizeof (sigq_t)));

          /* Check if we got an allocated message */

          if (sigq)
            {
              sigq->type = SIG_ALLOC_DYN;
            }
        }
    }

  return sigq;
}
