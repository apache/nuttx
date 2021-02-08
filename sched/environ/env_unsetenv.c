/****************************************************************************
 * sched/environ/env_unsetenv.c
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
 * Name: unsetenv
 *
 * Description:
 *   The unsetenv() function deletes the variable name from the environment.
 *
 * Input Parameters:
 *   name - The name of the variable to delete
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int unsetenv(FAR const char *name)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  FAR char *pvar;
  FAR char *newenvp;
  int newsize;
  int ret = OK;

  DEBUGASSERT(name && group);

  /* Check if the variable exists */

  sched_lock();
  if (group && (pvar = env_findvar(group, name)) != NULL)
    {
      /* It does!  Remove the name=value pair from the environment. */

      env_removevar(group, pvar);

      /* Reallocate the new environment buffer */

      newsize = group->tg_envsize;
      if (newsize <= 0)
        {
          /* Free the old environment (if there was one) */

          if (group->tg_envp != NULL)
            {
              kumm_free(group->tg_envp);
              group->tg_envp = NULL;
            }

          group->tg_envsize = 0;
        }
      else
        {
          /* Reallocate the environment to reclaim a little memory */

          newenvp = (FAR char *)kumm_realloc(group->tg_envp, newsize);
          if (newenvp == NULL)
            {
              set_errno(ENOMEM);
              ret = ERROR;
            }
          else
            {
              /* Save the new environment pointer (it might have changed due
               * to reallocation).
               */

              group->tg_envp = newenvp;
            }
        }
    }

  sched_unlock();
  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */
