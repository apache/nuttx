/****************************************************************************
 * sched/signal/sig_cleanup.c
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
  FAR sigq_t *sigq;

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

  while ((sigact = (FAR sigactq_t *)sq_remfirst(&group->tg_sigactionq))
         != NULL)
    {
      nxsig_release_action(sigact);
    }

  /* Deallocate all entries in the list of pending signals */

  while ((sigpend = (FAR sigpendq_t *)sq_remfirst(&group->tg_sigpendingq))
         != NULL)
    {
      nxsig_release_pendingsignal(sigpend);
    }
}
