/****************************************************************************
 * sched/sched/sched_get_stackinfo.c
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
#include <assert.h>

#include "nuttx/sched.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_stackinfo
 *
 * Description:
 *   Report information about a thread's stack allocation.
 *
 * Input Parameters:
 *   pid       - Identifies the thread to query.  Zero is interpreted as the
 *               the calling thread
 *   stackinfo - User-provided location to return the stack information.
 *
 * Returned Value:
 *   Zero (OK) if successful.  Otherwise, a negated errno value is returned.
 *
 *     -ENOENT  Returned if pid does not refer to an active task
 *     -EACCES  The calling thread does not have privileges to access the
 *              stack of the thread associated with the pid.
 *
 ****************************************************************************/

int nxsched_get_stackinfo(pid_t pid, FAR struct stackinfo_s *stackinfo)
{
  FAR struct tcb_s *rtcb = this_task();  /* TCB of running task */
  FAR struct tcb_s *qtcb;                /* TCB of queried task */

  DEBUGASSERT(rtcb != NULL && stackinfo != NULL);

  /*  Pid of 0 means that we are querying ourself */

  if (pid == 0)
    {
      /* We can always query ourself */

      qtcb = rtcb;
    }
  else
    {
      /* Get the task to be queried */

      qtcb = nxsched_get_tcb(pid);
      if (qtcb == NULL)
        {
          return -ENOENT;
        }

      /* A kernel thread can query any other thread.  Application threads
       * can only query application threads in the same task group.
       */

      if ((rtcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
        {
          /* It is an application thread.  It is permitted to query
           * only threads within the same task group.  It is not permitted
           * to peek into the stacks of either kernel threads or other
           * applications tasks.
           */

          if (rtcb->group != qtcb->group)
            {
              return -EACCES;
            }
        }
    }

  stackinfo->adj_stack_size  = qtcb->adj_stack_size;
  stackinfo->stack_alloc_ptr = qtcb->stack_alloc_ptr;
  stackinfo->adj_stack_ptr   = qtcb->adj_stack_ptr;
  return OK;
}
