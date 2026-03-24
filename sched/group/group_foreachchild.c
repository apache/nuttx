/****************************************************************************
 * sched/group/group_foreachchild.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <nuttx/nuttx.h>
#include <nuttx/sched.h>
#include <nuttx/queue.h>

#include "group/group.h"

#ifdef HAVE_GROUP_MEMBERS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_foreachchild
 *
 * Description:
 *   Execute a function for each child of a group.
 *
 * Input Parameters:
 *   group - The group containing the children
 *   handler - The function to be called
 *   arg - An additional argument to provide to the handler
 *
 * Returned Value:
 *   Success (OK) is always returned unless the handler returns a non-zero
 *   value (a negated errno on errors).  In that case, the traversal
 *   terminates and that non-zero value is returned.
 *
 * Assumptions:
 *
 ****************************************************************************/

int group_foreachchild(FAR struct task_group_s *group,
                       foreachchild_t handler, FAR void *arg)
{
  FAR sq_entry_t *curr;
  FAR sq_entry_t *next;
  int ret = OK;

  DEBUGASSERT(group);

  /* Visit the main thread last (if present) */

  sq_for_every_safe(&group->tg_members, curr, next)
    {
      FAR struct tcb_s *mtcb =
        container_of(curr, struct tcb_s, member);

      ret = handler(mtcb->pid, arg);
      if (ret != OK)
        {
          break;
        }
    }

  return ret;
}

#endif /* HAVE_GROUP_MEMBERS */
