/****************************************************************************
 * sched/sched/sched_suspendscheduler.c
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

#include <time.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/sched_note.h>

#include "clock/clock.h"
#include "sched/sched.h"

#ifdef CONFIG_SCHED_SUSPENDSCHEDULER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_suspend_scheduler
 *
 * Description:
 *   Called by architecture specific implementations that starts task
 *   execution.  This function prepares the scheduler for the thread that is
 *   about to be restarted.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that is being suspended.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_suspend_scheduler(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SCHED_SPORADIC
  /* Perform sporadic schedule operations */

  if ((tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      DEBUGVERIFY(nxsched_suspend_sporadic(tcb));
    }
#endif

  /* Indicate that the task has been suspended */

#ifdef CONFIG_SCHED_CRITMONITOR
  nxsched_suspend_critmon(tcb);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION
  sched_note_suspend(tcb);
#endif
}

#endif /* CONFIG_SCHED_SUSPENDSCHEDULER */
