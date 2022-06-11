/****************************************************************************
 * sched/pthread/pthread_detach.c
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
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "sched/sched.h"
#include "group/group.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_detach
 *
 * Description:
 *    A thread object may be "detached" to specify that the return value
 *    and completion status will not be requested.
 *
 *    The caller's task/thread must belong to the same "task group" as the
 *    pthread is (or was) a member of.  The thread may or may not still
 *    be running.
 *
 * Input Parameters:
 *   thread
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_detach(pthread_t thread)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  FAR struct join_s *pjoin;
  int ret = OK;

  sinfo("Thread=%d group=%p\n", thread, group);
  DEBUGASSERT(group);

  /* Find the entry associated with this pthread. */

  nxsem_wait_uninterruptible(&group->tg_joinsem);
  pjoin = pthread_findjoininfo(group, (pid_t)thread);
  if (!pjoin)
    {
      serr("ERROR: Could not find thread entry\n");
      ret = EINVAL;
    }
  else
    {
      /* Has the thread already terminated? */

      if (pjoin->terminated)
        {
          /* YES.. just remove the thread entry. */

          pthread_destroyjoin(group, pjoin);
        }
      else
        {
          /* NO.. Just mark the thread as detached.  It
           * will be removed and deallocated when the
           * thread exits
           */

          if (pjoin->detached)
            {
              ret = EINVAL;
            }
          else
            {
              pjoin->detached = true;
            }
        }
    }

  pthread_sem_give(&group->tg_joinsem);

  sinfo("Returning %d\n", ret);
  return ret;
}
