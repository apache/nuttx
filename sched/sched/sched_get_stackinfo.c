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
 * Name: sched_get_stackinfo
 *
 * Description:
 *   Report information about a thread's stack allocation.
 *
 * Input Parameters:
 *   pid_t     - Identifies the thread to query.  Zero is interpreted as the
 *               the calling thread
 *   stackinfo - User-provided location to return the stack information.
 *
 * Returned Value:
 *   Zero (OK) if successful.  Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int sched_get_stackinfo(pid_t pid, FAR struct stackinfo_s *stackinfo)
{
  FAR struct tcb_s *tcb;

  DEBUGASSERT(stackinfo != NULL);

  if (pid == 0)
    {
      tcb = this_task();
      DEBUGASSERT(tcb != NULL);
    }
  else
    {
      tcb = sched_gettcb(pid);
      if (tcb == NULL)
        {
          return -ENOENT;
        }
    }

  stackinfo->adj_stack_size  = tcb->adj_stack_size;
  stackinfo->stack_alloc_ptr = tcb->stack_alloc_ptr;
  stackinfo->adj_stack_ptr   = tcb->adj_stack_ptr;
  return OK;
}
