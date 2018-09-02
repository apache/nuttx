/****************************************************************************
 *  sched/group/group_killchildren.c
 *
 *   Copyright (C) 2013, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Name: group_killchildren_handler
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

static int group_killchildren_handler(pid_t pid, FAR void *arg)
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

      rtcb = sched_gettcb(pid);
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

  /* Always return zero.  We need to visit each member of the group*/

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_killchildren
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

int group_killchildren(FAR struct task_tcb_s *tcb)
{
  int ret;

  /* Lock the scheduler so that there this thread will not lose priority
   * until all of its children are suspended.
   */

  sched_lock();
  ret = group_foreachchild(tcb->cmn.group, group_killchildren_handler,
                          (FAR void *)((uintptr_t)tcb->cmn.pid));
  sched_unlock();
  return ret;
}

#endif /* HAVE_GROUP_MEMBERS */
