/****************************************************************************
 * libs/libc/environ/env_dup.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/mutex.h>
#include <nuttx/tls.h>

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
 *   info  - The child task to receive the newly allocated copy of
 *           the parent task groups environment structure.
 *   envcp - Pointer to the environment strings to copy.
 * Returned Value:
 *   zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

int env_dup(FAR struct task_info_s *info, FAR char * const *envcp)
{
  FAR char **envp;
  FAR char *str;
  size_t envc = 0;
  size_t size = 0;
  size_t i;
  int ret = OK;

  DEBUGASSERT(info != NULL);

  /* Is there an environment ? */

  if (envcp != NULL && info->ta_envp == NULL)
    {
      nxrmutex_lock(&info->ta_lock);

      /* Count the strings and their size */

      while (envcp[envc] != NULL)
        {
          size += strlen(envcp[envc++]) + 1;
        }

      /* A special case is that the parent has an "empty" environment
       * allocation, i.e., there is an allocation in place but it
       * contains no variable definitions and, hence, envc == 0.
       */

      if (envc > 0)
        {
          /* Duplicate the parent environment: one block for the array and
           * one for all of the strings.
           */

          info->ta_envpc = envc + SCHED_ENVIRON_RESERVED + 1;
          envp = env_alloc(info, sizeof(*envp) * info->ta_envpc);
          str  = env_alloc(info, size);
          if (envp == NULL || str == NULL)
            {
              /* The parent's environment can not be inherited due to a
               * failure in the allocation of the child environment.
               */

              if (envp != NULL)
                {
                  env_free(info, envp);
                }

              if (str != NULL)
                {
                  env_free(info, str);
                }

              info->ta_envpc = 0;
              ret = -ENOMEM;
            }
          else
            {
              for (i = 0; i < envc; i++)
                {
                  size = strlen(envcp[i]) + 1;
                  memcpy(str, envcp[i], size);
                  envp[i] = str;
                  str += size;
                }

              envp[envc]      = NULL;
              info->ta_envp   = envp;
              info->ta_envarr = envp;
              info->ta_envc   = envc;
            }
        }

      nxrmutex_unlock(&info->ta_lock);
    }

  return ret;
}
