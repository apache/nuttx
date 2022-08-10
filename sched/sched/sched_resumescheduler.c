/****************************************************************************
 * sched/sched/sched_resumescheduler.c
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

#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "sched/sched.h"

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_RESUMESCHEDULER)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_resume_scheduler
 *
 * Description:
 *   Called by architecture specific implementations that block task
 *   execution.  This function prepares the scheduler for the thread that is
 *   about to be restarted.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread to be restarted.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_resume_scheduler(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SCHED_SPORADIC
  if ((tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      /* Reset the replenishment cycle if it is appropriate to do so */

      DEBUGVERIFY(nxsched_resume_sporadic(tcb));
    }
#endif

  /* Indicate the task has been resumed */

#ifdef CONFIG_SCHED_CRITMONITOR
  nxsched_resume_critmon(tcb);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION
  sched_note_resume(tcb);
#endif
}

#endif /* CONFIG_RR_INTERVAL > 0 || CONFIG_SCHED_RESUMESCHEDULER */
