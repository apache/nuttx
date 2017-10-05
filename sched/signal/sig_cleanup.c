/****************************************************************************
 * sched/signal/sig_cleanup.c
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
#include <nuttx/arch.h>

#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_cleanup
 *
 * Description:
 *   Deallocate all signal-related lists in a TCB.  This function is
 *   called only at task deletion time.  The caller is expected to have
 *   assured the critical section necessary to perform this action.
 *
 ****************************************************************************/

void nxsig_cleanup(FAR struct tcb_s *stcb)
{
  FAR sigq_t     *sigq;

  /* Deallocate all entries in the list of pending signal actions */

  while ((sigq = (FAR sigq_t *)sq_remfirst(&stcb->sigpendactionq)) != NULL)
    {
      nxsig_release_pendingsigaction(sigq);
    }

  /* Deallocate all entries in the list of posted signal actions */

  while ((sigq = (FAR sigq_t *)sq_remfirst(&stcb->sigpostedq)) != NULL)
    {
      nxsig_release_pendingsigaction(sigq);
    }

  /* Misc. signal-related clean-up */

  stcb->sigprocmask  = ALL_SIGNAL_SET;
  stcb->sigwaitmask  = NULL_SIGNAL_SET;
}

/****************************************************************************
 * Name: nxsig_release
 *
 * Description:
 *   Deallocate all signal-related lists in a group.  This function is
 *   called only when the last thread leaves the group.  The caller is
 *   expected to have assured the critical section necessary to perform
 *   this action.
 *
 ****************************************************************************/

void nxsig_release(FAR struct task_group_s *group)
{
  FAR sigactq_t  *sigact;
  FAR sigpendq_t *sigpend;

  /* Deallocate all entries in the list of signal actions */

  while ((sigact = (FAR sigactq_t *)sq_remfirst(&group->tg_sigactionq)) != NULL)
    {
      nxsig_release_action(sigact);
    }

  /* Deallocate all entries in the list of pending signals */

  while ((sigpend = (FAR sigpendq_t *)sq_remfirst(&group->tg_sigpendingq)) != NULL)
    {
      nxsig_release_pendingsignal(sigpend);
    }
}

