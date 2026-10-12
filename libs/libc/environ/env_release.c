/****************************************************************************
 * libs/libc/environ/env_release.c
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

#include <assert.h>

#include <nuttx/tls.h>

#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_release
 *
 * Description:
 *   env_release() is called only from task_uninit_info() when the task is
 *   uninit. The env_release() function clears the environment of all
 *   name-value pairs and sets the value of the external variable
 *   environ to NULL.
 *
 * Input Parameters:
 *   info - Identifies the task containing the environment structure
 *          to be released.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

void env_release(FAR struct task_info_s *info)
{
  int i;

  DEBUGASSERT(info != NULL);

  if (info->ta_envp)
    {
      /* Free any allocate environment strings */

      for (i = 0; info->ta_envp[i] != NULL; i++)
        {
          env_free(info, info->ta_envp[i]);
        }

      /* Free the environment */

      env_free(info, info->ta_envp);
    }

  /* In any event, make sure that all environment-related variables in the
   * task group structure are reset to initial values.
   */

  info->ta_envp  = NULL;
  info->ta_envpc = 0;
  info->ta_envc  = 0;
}
