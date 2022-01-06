/****************************************************************************
 * sched/signal/sig_pending.c
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
#include <sched.h>
#include <assert.h>

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
