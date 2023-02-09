/****************************************************************************
 * sched/environ/env_release.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_release
 *
 * Description:
 *   env_release() is called only from group_leave() when the last member of
 *   a task group exits.  The env_release() function clears the environment
 *   of all name-value pairs and sets the value of the external variable
 *   environ to NULL.
 *
 * Input Parameters:
 *   group - Identifies the task group containing the environment structure
 *           to be released.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

void env_release(FAR struct task_group_s *group)
{
  int i;

  DEBUGASSERT(group != NULL);

  if (group->tg_envp)
    {
      /* Free any allocate environment strings */

      for (i = 0; group->tg_envp[i] != NULL; i++)
        {
          group_free(group, group->tg_envp[i]);
        }

      /* Free the environment */

      group_free(group, group->tg_envp);
    }

  /* In any event, make sure that all environment-related variables in the
   * task group structure are reset to initial values.
   */

  group->tg_envp = NULL;
  group->tg_envc = 0;
}

#endif /* CONFIG_DISABLE_ENVIRON */
