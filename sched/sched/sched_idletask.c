/****************************************************************************
 * sched/sched/sched_idletask.c
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

#include <stdbool.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_idletask
 *
 * Description:
 *   Check if the caller is an IDLE thread.  For most implementations of
 *   the SYSLOG output semaphore locking is required for mutual exclusion.
 *   The idle threads are unable to lock semaphores because they cannot
 *   wait.  So IDLE thread output is a special case and is treated much as
 *   we treat debug output from an interrupt handler.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if the calling task is and IDLE thread.
 *
 ****************************************************************************/

bool sched_idletask(void)
{
  FAR struct tcb_s *rtcb = this_task();

  /* If called early in the initialization sequence, the tasks lists may not
   * have been initialized and, in that case, rtcb may be NULL.
   */

  DEBUGASSERT(rtcb != NULL || g_nx_initstate < OSINIT_TASKLISTS);
  if (rtcb != NULL)
    {
      /* The IDLE task TCB is distinguishable by a few things:
       *
       * (1) It always lies at the end of the task list,
       * (2) It always has priority zero, and
       * (3) It should have the TCB_FLAG_CPU_LOCKED flag set.
       *
       * In the non-SMP case, the IDLE task will also have PID=0, but that
       * is not a portable test because there are multiple IDLE tasks with
       * different PIDs in the SMP configuration.
       */

      return (rtcb->flink == NULL);
    }

  /* We must be on the IDLE thread if we are early in initialization */

  return true;
}
