/****************************************************************************
 * sched/environ/env_foreach.c
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

#include <stdbool.h>
#include <string.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/environ.h>

#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_foreach
 *
 * Description:
 *   Visit each name-value pair in the environment.
 *
 * Input Parameters:
 *   group - The task group containing environment array to be searched.
 *   cb    - The callback function to be invoked for each environment
 *           variable.
 *
 * Returned Value:
 *   Zero if the all environment variables have been traversed.  A non-zero
 *   value means that the callback function requested early termination by
 *   returning a nonzero value.
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emptions is disabled by caller
 *
 ****************************************************************************/

int env_foreach(FAR struct task_group_s *group,
                env_foreach_t cb,
                FAR void *arg)
{
  int ret = OK;
  int i;

  /* Verify input parameters */

  DEBUGASSERT(group != NULL && cb != NULL);

  if (group->tg_envp == NULL)
    {
      return ret;
    }

  for (i = 0; group->tg_envp[i] != NULL; i++)
    {
      /* Perform the callback */

      ret = cb(arg, group->tg_envp[i]);

      /* Terminate the traversal early if the callback so requests by
       * returning a non-zero value.
       */

      if (ret != 0)
        {
          break;
        }
    }

  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */
