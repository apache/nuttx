/****************************************************************************
 * sched/group/group_killchildren.c
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
#include <debug.h>

#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"

#ifdef HAVE_GROUP_MEMBERS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_kill_children_handler
 *
 * Description:
 *   Callback from group_foreachchild that handles one member of the group.
 *
 * Input Parameters:
 *   pid - The ID of the group member that may be signaled.
 *   arg - The PID of the thread to be retained.
 *
 * Returned Value:
 *   0 (OK) always
 *
 ****************************************************************************/

static int group_kill_children_handler(pid_t pid, FAR void *arg)
{
  FAR struct tcb_s *rtcb;
  int ret;

  /* Cancel all threads except for the one specified by the argument */

  if (pid != (pid_t)((uintptr_t)arg))
    {
      /* Cancel this thread.  This is a forced cancellation.  Make sure that
       * cancellation is not disabled by the task/thread.  That bit will
       * prevent pthread_cancel() or task_delete() from doing what they need
       * to do.
       */

      rtcb = nxsched_get_tcb(pid);
      if (rtcb != NULL)
        {
          /* This is a forced cancellation.  Make sure that cancellation is
           * not disabled by the task/thread.  That bit would prevent
           * pthread_cancel() or task_delete() from doing what they need
           * to do.
           */

          rtcb->flags &= ~TCB_FLAG_NONCANCELABLE;

          /* 'pid' could refer to the main task of the thread.  That pid
           * will appear in the group member list as well!
           */

          if ((rtcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
            {
              ret = pthread_cancel(pid);
            }
          else
            {
              ret = task_delete(pid);
            }

          if (ret < 0)
            {
              serr("ERROR: Failed to kill %d: %d\n", ret, pid);
            }
        }
    }

  /* Always return zero.  We need to visit each member of the group. */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_kill_children
 *
 * Description:
 *   Delete all children of a task except for the specified task.  This is
 *   used by the task restart logic and by the default signal handling
 *   abnormal termination logic.  When the main task is restarted or killed,
 *   all of its child pthreads must be terminated.
 *
 * Input Parameters:
 *   tcb - TCB of the task to be retained.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int group_kill_children(FAR struct tcb_s *tcb)
{
  int ret;

#ifdef CONFIG_SMP
  /* NOTE: sched_lock() is not enough for SMP
   * because tcb->group will be accessed from the child tasks
   */

  irqstate_t flags = enter_critical_section();
#else
  /* Lock the scheduler so that there this thread will not lose priority
   * until all of its children are suspended.
   */

  sched_lock();
#endif
  ret = group_foreachchild(tcb->group, group_kill_children_handler,
                           (FAR void *)((uintptr_t)tcb->pid));
#ifdef CONFIG_SMP
  leave_critical_section(flags);
#else
  sched_unlock();
#endif
  return ret;
}

#endif /* HAVE_GROUP_MEMBERS */
