/****************************************************************************
 * sched/signal/sig_releasependingsigaction.c
 *
 *   Copyright (C) 2007, 2009, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

  /* If this is a generally available pre-allocated structyre,
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
