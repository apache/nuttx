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
#include <assert.h>
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
 *   group - The child task group to receive the newly allocated copy of
 *           the parent task groups environment structure.
 *   envcp - Pointer to the environment strings to copy.
 * Returned Value:
 *   zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

int env_dup(FAR struct task_group_s *group, FAR char * const *envcp)
{
  FAR char **envp = NULL;
  size_t envc = 0;
  int ret = OK;

  DEBUGASSERT(group != NULL);

  /* Pre-emption must be disabled throughout the following because the
   * environment may be shared.
   */

  sched_lock();

  /* Is there an environment ? */

  if (envcp != NULL)
    {
      /* Count the strings */

      while (envcp[envc] != NULL)
        {
          envc++;
        }

      /* A special case is that the parent has an "empty" environment
       * allocation, i.e., there is an allocation in place but it
       * contains no variable definitions and, hence, envc == 0.
       */

      if (envc > 0)
        {
          /* There is an environment, duplicate it */

          envp = group_malloc(group, sizeof(*envp) * (envc + 1));
          if (envp == NULL)
            {
              /* The parent's environment can not be inherited due to a
               * failure in the allocation of the child environment.
               */

              ret = -ENOMEM;
            }
          else
            {
              envp[envc] = NULL;

              /* Duplicate the parent environment. */

              while (envc-- > 0)
                {
                  envp[envc] = group_malloc(group, strlen(envcp[envc]) + 1);
                  if (envp[envc] == NULL)
                    {
                      while (envp[++envc] != NULL)
                        {
                          group_free(group, envp[envc]);
                        }

                      group_free(group, envp);
                      sched_unlock();
                      return -ENOMEM;
                    }

                  strcpy(envp[envc], envcp[envc]);
                }
            }
        }

      /* Save the child environment allocation. */

      group->tg_envp = envp;
    }

  sched_unlock();
  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */
