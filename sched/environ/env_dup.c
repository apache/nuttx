/****************************************************************************
 * sched/environ/env_dup.c
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

#ifndef CONFIG_DISABLE_ENVIRON

#include <sys/types.h>
#include <sched.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "sched/sched.h"
#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_dup
 *
 * Description:
 *   Copy the internal environment structure of a task.  This is the action
 *   that is performed when a new task is created: The new task has a
 *   private, exact duplicate of the parent task's environment.
 *
 * Input Parameters:
 *   group - The child task group to receive the newly allocated copy of the
 *           parent task groups environment structure.
 *
 * Returned Value:
 *   zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

int env_dup(FAR struct task_group_s *group)
{
  FAR struct tcb_s *ptcb = this_task();
  FAR char *envp = NULL;
  size_t envlen;
  int ret = OK;

  DEBUGASSERT(group != NULL && ptcb != NULL && ptcb->group != NULL);

  /* Pre-emption must be disabled throughout the following because the
   * environment may be shared.
   */

  sched_lock();

  /* Does the parent task have an environment? */

  if (ptcb->group != NULL && ptcb->group->tg_envp != NULL)
    {
      /* Yes.. The parent task has an environment allocation. */

      envlen = ptcb->group->tg_envsize;
      envp   = NULL;

      /* A special case is that the parent has an "empty" environment
       * allocation, i.e., there is an allocation in place but it
       * contains no variable definitions and, hence, envlen == 0.
       */

      if (envlen > 0)
        {
          /* There is an environment, duplicate it */

          envp = (FAR char *)kumm_malloc(envlen);
          if (envp == NULL)
            {
              /* The parent's environment can not be inherited due to a
               * failure in the allocation of the child environment.
               */

              envlen = 0;
              ret    = -ENOMEM;
            }
          else
            {
              /* Duplicate the parent environment. */

              memcpy(envp, ptcb->group->tg_envp, envlen);
            }
        }

      /* Save the size and child environment allocation. */

      group->tg_envsize = envlen;
      group->tg_envp    = envp;
    }

  sched_unlock();
  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */
