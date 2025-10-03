/****************************************************************************
 * sched/sched/sched_switchcontext.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "sched/sched.h"

#include <nuttx/sched_note.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_switch_context
 *
 * Description:
 *   This function is used to switch context between two tasks.
 *
 * Input Parameters:
 *   from - The TCB of the task to be suspended.
 *   to   - The TCB of the task to be resumed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_switch_context(FAR struct tcb_s *from, FAR struct tcb_s *to)
{
  nxsched_checkstackoverflow(from);

#ifdef CONFIG_SCHED_SPORADIC
  /* Perform sporadic schedule operations */

  if ((from->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      DEBUGVERIFY(nxsched_suspend_sporadic(from));
    }

  if ((to->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      DEBUGVERIFY(nxsched_resume_sporadic(to));
    }
#endif

  /* Indicate that the task has been suspended */

#ifdef CONFIG_SCHED_CRITMONITOR
  nxsched_switch_critmon(from, to);
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION
  sched_note_suspend(from);
  sched_note_resume(to);
#endif
}
