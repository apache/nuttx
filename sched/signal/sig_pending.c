/****************************************************************************
 * sched/signal/sig_pending.c
 *
 *   Copyright (C) 2007-2009, 2013, 2016 Gregory Nutt. All rights reserved.
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

#include <signal.h>
#include <sched.h>

#include <nuttx/irq.h>
#include <nuttx/signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigpending
 *
 * Description:
 *   This function returns the set of signals that are blocked from deliveryi
 *   and that are pending for the calling process in the space pointed to by
 *   set.
 *
 * Input Parameters:
 *   set - The location to return the pending signal set.
 *
 * Returned Value:
 *   0 (OK) or -1 (ERROR)
 *
 * Assumptions:
 *
 ****************************************************************************/

int sigpending(FAR sigset_t *set)
{
  FAR struct tcb_s *rtcb = this_task();
  int ret = ERROR;

  if (set)
    {
      *set = nxsig_pendingset(rtcb);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: nxsig_pendingset
 *
 * Description:
 *   Convert the list of pending signals into a signal set
 *
 ****************************************************************************/

sigset_t nxsig_pendingset(FAR struct tcb_s *stcb)
{
  FAR struct task_group_s *group = stcb->group;
  sigset_t sigpendset;
  FAR sigpendq_t *sigpend;
  irqstate_t flags;

  DEBUGASSERT(group);

  sigpendset = NULL_SIGNAL_SET;

  flags = enter_critical_section();
  for (sigpend = (FAR sigpendq_t *)group->tg_sigpendingq.head;
       (sigpend); sigpend = sigpend->flink)
    {
      nxsig_addset(&sigpendset, sigpend->info.si_signo);
    }

  leave_critical_section(flags);

  return sigpendset;
}
