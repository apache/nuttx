/****************************************************************************
 * sched/signal/sig_findaction.c
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
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_find_action
 *
 * Description:
 *   Allocate a new element for a signal queue
 *
 ****************************************************************************/

FAR sigactq_t *nxsig_find_action(FAR struct task_group_s *group, int signo)
{
  FAR sigactq_t *sigact = NULL;

  /* Verify the caller's sanity */

  if (group)
    {
      /* Sigactions can only be assigned to the currently executing
       * thread.  So, a simple lock ought to give us sufficient
       * protection.
       */

      sched_lock();

      /* Search the list for a sigaction on this signal */

      for (sigact = (FAR sigactq_t *)group->tg_sigactionq.head;
           ((sigact) && (sigact->signo != signo));
           sigact = sigact->flink);

      sched_unlock();
    }

  return sigact;
}
