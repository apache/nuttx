/****************************************************************************
 * sched/group/group_waiter.c
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

#include <sys/types.h>

#include <assert.h>

#include "nuttx/sched.h"
#include "group/group.h"

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  group_add_waiter
 *
 * Description:
 *   Increment the number of instances of waitpid that are waiting for this
 *   group to exit.
 *
 * Assumptions:
 *   Caller has assured mutually exclusive access to the group.
 *
 ****************************************************************************/

void group_add_waiter(FAR struct task_group_s *group)
{
  group->tg_nwaiters++;
  DEBUGASSERT(group->tg_nwaiters > 0);
}

/****************************************************************************
 * Name:  group_add_waiter
 *
 * Description:
 *   Decrement the number of instances of waitpid that are waiting for this
 *   group to exit.  If the count goes to zero and deletion is pending, the
 *   call group_free to release the dangling resources.
 *
 * Assumptions:
 *   Caller has assured mutually exclusive access to the group.
 *
 ****************************************************************************/

void group_del_waiter(FAR struct task_group_s *group)
{
  DEBUGASSERT(group->tg_nwaiters > 0);
  group->tg_nwaiters--;
  if (group->tg_nwaiters == 0 && (group->tg_flags & GROUP_FLAG_DELETED) != 0)
    {
      /* Release the group container (all other resources have already been
       * freed).
       */

      group_deallocate(group);
    }
}

#endif /* CONFIG_SCHED_WAITPID && !CONFIG_SCHED_HAVE_PARENT */
