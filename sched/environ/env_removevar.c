/****************************************************************************
 * sched/environ/env_removevar.c
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

#ifndef CONFIG_DISABLE_ENVIRON

#include <string.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/kmalloc.h>

#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_removevar
 *
 * Description:
 *   Remove the referenced name=value pair from the environment
 *
 * Input Parameters:
 *   group - The task group with the environment containing the name=value
 *           pair
 *   index - A index to the name=value pair in the restroom
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Caller has pre-emption disabled
 *   - Caller will reallocate the environment structure to the correct size
 *
 ****************************************************************************/

void env_removevar(FAR struct task_group_s *group, ssize_t index)
{
  DEBUGASSERT(group != NULL && index >= 0 && index < group->tg_envc);

  /* Free the allocate environment string */

  group_free(group, group->tg_envp[index]);

  /* Exchange the last env and the index env */

  group->tg_envc--;
  if (index == group->tg_envc)
    {
      group->tg_envp[index] = NULL;
    }
  else
    {
      group->tg_envp[index] = group->tg_envp[group->tg_envc];
      group->tg_envp[group->tg_envc] = NULL;
    }

  /* Free the old environment (if there was one) */

  if (group->tg_envc == 0)
    {
      group_free(group, group->tg_envp);
      group->tg_envp = NULL;
      group->tg_envpc = 0;
    }
  else if (group->tg_envc <=
           (group->tg_envpc - SCHED_ENVIRON_RESERVED * 2))
    {
      /* Reallocate the environment to reclaim a little memory */

      group->tg_envpc = group->tg_envc + SCHED_ENVIRON_RESERVED + 1;

      group->tg_envp = group_realloc(group, group->tg_envp,
         sizeof(*group->tg_envp) * group->tg_envpc);
      DEBUGASSERT(group->tg_envp != NULL);
    }
}

#endif /* CONFIG_DISABLE_ENVIRON */
