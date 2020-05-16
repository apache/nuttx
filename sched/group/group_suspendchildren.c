/****************************************************************************
 *  sched/group/group_suspendchildren.c
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

#include <sys/types.h>
#include <stdint.h>
#include <sched.h>
#include <pthread.h>

#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"

#ifdef HAVE_GROUP_MEMBERS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_suspend_children_handler
 *
 * Description:
 *   Callback from group_foreachchild that handles one member of the group.
 *
 * Input Parameters:
 *   pid - The ID of the group member that may be suspended.
 *   arg - The PID of the thread to be retained.
 *
 * Returned Value:
 *   0 (OK) always
 *
 ****************************************************************************/

static int group_suspend_children_handler(pid_t pid, FAR void *arg)
{
  FAR struct tcb_s *rtcb;

  /* Suspend all threads except for the one specified by the argument */

  if (pid != (pid_t)((uintptr_t)arg))
    {
      /* Suspend this thread if it is still alive. */

      rtcb = nxsched_get_tcb(pid);
      if (rtcb != NULL)
        {
          nxsched_suspend(rtcb);
        }
    }

  /* Always return zero.  We need to visit each member of the group. */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_suspend_children
 *
 * Description:
 *   Suspend all children of a task except for the specified task.  This is
 *   SIGSTP/SIGSTOP default signal action logic.  When the main task is
 *   suspended, all of its child pthreads must also be suspended.
 *
 * Input Parameters:
 *   tcb - TCB of the task to be retained.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int group_suspend_children(FAR struct tcb_s *tcb)
{
  int ret;

  /* Lock the scheduler so that there this thread will not lose priority
   * until all of its children are suspended.
   */

  sched_lock();
  ret = group_foreachchild(tcb->group, group_suspend_children_handler,
                           (FAR void *)((uintptr_t)tcb->pid));
  sched_unlock();
  return ret;
}

#endif /* HAVE_GROUP_MEMBERS */
