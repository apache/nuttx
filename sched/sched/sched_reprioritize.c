/****************************************************************************
 * sched/sched/sched_reprioritize.c
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

#include <stdint.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "sched/sched.h"

#ifdef CONFIG_PRIORITY_INHERITANCE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_reprioritize
 *
 * Description:
 *   This function sets the priority of a specified task.
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to  after all other tasks
 *   with the same priority.
 *
 * Input Parameters:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Returned Value:
 *   On success, sched_reporioritize() returns 0 (OK). On error, a negated
 *   errno value is returned.
 *
 *   EINVAL The parameter 'param' is invalid or does not make sense for the
 *          current scheduling policy.
 *   EPERM  The calling task does not have appropriate privileges.
 *   ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int nxsched_reprioritize(FAR struct tcb_s *tcb, int sched_priority)
{
  /* This function is equivalent to nxsched_set_priority() BUT it also has
   * the side effect of discarding all priority inheritance history.  This
   * is done only on explicit, user-initiated reprioritization.
   */

  int ret = nxsched_set_priority(tcb, sched_priority);
  if (ret == 0)
    {
      /* Reset the base_priority -- the priority that the thread would return
       * to once it posts the semaphore.
       */

      tcb->base_priority  = (uint8_t)sched_priority;

      /* Discard priority boost as well */

      tcb->boost_priority = 0;
    }

  return ret;
}
#endif /* CONFIG_PRIORITY_INHERITANCE */
