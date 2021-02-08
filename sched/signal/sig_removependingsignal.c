/****************************************************************************
 * sched/signal/sig_removependingsignal.c
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

#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>

#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_remove_pendingsignal
 *
 * Description:
 *   Remove the specified signal from the signal pending list
 *
 ****************************************************************************/

FAR sigpendq_t *nxsig_remove_pendingsignal(FAR struct tcb_s *stcb, int signo)
{
  FAR struct task_group_s *group = stcb->group;
  FAR sigpendq_t *currsig;
  FAR sigpendq_t *prevsig;
  irqstate_t  flags;

  DEBUGASSERT(group);

  flags = enter_critical_section();

  for (prevsig = NULL,
       currsig = (FAR sigpendq_t *)group->tg_sigpendingq.head;
       (currsig && currsig->info.si_signo != signo);
       prevsig = currsig, currsig = currsig->flink);

  if (currsig)
    {
      if (prevsig)
        {
          sq_remafter((FAR sq_entry_t *)prevsig, &group->tg_sigpendingq);
        }
      else
        {
          sq_remfirst(&group->tg_sigpendingq);
        }
    }

  leave_critical_section(flags);

  return currsig;
}
