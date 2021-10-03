/****************************************************************************
 * sched/signal/sig_default.c
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
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <sys/wait.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <nuttx/signal.h>
#include <nuttx/tls.h>

#include "group/group.h"
#include "sched/sched.h"
#include "task/task.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_abnormal_termination
 *
 * Description:
 *   This is the handler for the abnormal termination default action.
 *
 * Input Parameters:
 *   Standard signal handler parameters
 *
 * Returned Value:
 *   int - Thread type
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_DEFAULT
int nxsig_abnormal_termination(int signo)
{
  FAR struct tcb_s *rtcb = (FAR struct tcb_s *)this_task();

  /* Careful:  In the multi-threaded task, the signal may be handled on a
   * child pthread.
   */

#ifdef HAVE_GROUP_MEMBERS
  /* Kill all of the children of the task.  This will not kill the currently
   * running task/pthread (this_task).  It will kill the main thread of the
   * task group if this_task is a pthread.
   */

  group_kill_children(rtcb);
#endif

  return rtcb->flags & TCB_FLAG_TTYPE_MASK;
}
#endif

/****************************************************************************
 * Name: nxsig_stop_task
 *
 * Description:
 *   This is the handler for the stop default action.
 *
 * Input Parameters:
 *   Standard signal handler parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_SIGSTOP_ACTION
void nxsig_stop_task(int signo)
{
  FAR struct tcb_s *rtcb = (FAR struct tcb_s *)this_task();
#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  FAR struct task_group_s *group;

  DEBUGASSERT(rtcb != NULL && rtcb->group != NULL);
  group = rtcb->group;
#endif

  /* Careful:  In the multi-threaded task, the signal may be handled on a
   * child pthread.
   */

#ifdef HAVE_GROUP_MEMBERS
  /* Suspend all of the children of the task.  This will not suspend the
   * currently running task/pthread (this_task).  It will suspend the
   * main thread of the task group if this_task is a pthread.
   */

  group_suspend_children(rtcb);
#endif

  /* Lock the scheduler so this thread is not pre-empted until after we
   * call nxsched_suspend().
   */

  sched_lock();

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  /* Notify via waitpid if any parent is waiting for this task to EXIT
   * or STOP.  This action is only performed if WUNTRACED is set in the
   * waitpid flags.
   */

  if ((group->tg_waitflags & WUNTRACED) != 0)
    {
      /* Return zero for exit status (we are not exiting, however) */

      if (group->tg_statloc != NULL)
        {
          *group->tg_statloc = 0;
           group->tg_statloc = NULL;
        }

      /* tg_waitflags == 0 means that the flags are available to another
       * caller of waitpid().
       */

      group->tg_waitflags = 0;

      /* Wakeup any tasks waiting for this task to exit or stop. */

      while (group->tg_exitsem.semcount < 0)
        {
          /* Wake up the thread */

          nxsem_post(&group->tg_exitsem);
        }
    }
#endif

  /* Then, finally, suspend this the final thread of the task group */

  nxsched_suspend(rtcb);
  sched_unlock();
}
#endif
