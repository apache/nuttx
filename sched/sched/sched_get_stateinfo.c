/****************************************************************************
 * sched/sched/sched_get_stateinfo.c
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

#include <string.h>
#include <stdio.h>
#include <semaphore.h>
#include <nuttx/mutex.h>
#include <nuttx/irq.h>
#include "nuttx/sched.h"

/****************************************************************************
 * Pre-processor types
 ****************************************************************************/

/* This is the state info of the task_state field of the TCB */

static FAR const char * const g_statenames[] =
{
  "Invalid",
  "Waiting,Unlock",
  "Ready",
#ifdef CONFIG_SMP
  "Assigned",
#endif
  "Running",
  "Inactive",
  "Waiting,Semaphore",
  "Waiting,Signal"
#if !defined(CONFIG_DISABLE_MQUEUE) || !defined(CONFIG_DISABLE_MQUEUE_SYSV)
  , "Waiting,MQ empty"
  , "Waiting,MQ full"
#endif
#ifdef CONFIG_PAGING
  , "Waiting,Paging fill"
#endif
#ifdef CONFIG_SIG_SIGSTOP_ACTION
  , "Stopped"
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_stateinfo
 *
 * Description:
 *   Report information about a thread's state
 *
 * Input Parameters:
 *   tcb    - The TCB for the task (same as the nxtask_init argument).
 *   state  - User-provided location to return the state information.
 *   length - The size of the state
 *
 ****************************************************************************/

void nxsched_get_stateinfo(FAR struct tcb_s *tcb, FAR char *state,
                           size_t length)
{
  irqstate_t flags;

  /* if the state is waiting mutex */

  flags = enter_critical_section();

  if (tcb->task_state == TSTATE_WAIT_SEM &&
      ((FAR sem_t *)(tcb->waitobj))->flags & SEM_TYPE_MUTEX)
    {
      pid_t holder = ((FAR mutex_t *)(tcb->waitobj))->holder;
      leave_critical_section(flags);

      snprintf(state, length, "Waiting,Mutex:%d", holder);
    }
  else
    {
      leave_critical_section(flags);
      strlcpy(state, g_statenames[tcb->task_state], length);
    }
}
