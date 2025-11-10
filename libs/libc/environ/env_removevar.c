/****************************************************************************
 * libs/libc/environ/env_removevar.c
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
 * Name: env_removevar
 *
 * Description:
 *   Remove the referenced name=value pair from the environment
 *
 * Input Parameters:
 *   info  - The task with the environment containing the name=value
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

void env_removevar(FAR struct task_info_s *info, ssize_t index)
{
  DEBUGASSERT(info != NULL && index >= 0 && index < info->ta_envc);

  /* Free the allocate environment string */

  lib_ufree(info->ta_envp[index]);

  /* Exchange the last env and the index env */

  info->ta_envc--;
  if (index == info->ta_envc)
    {
      info->ta_envp[index] = NULL;
    }
  else
    {
      info->ta_envp[index] = info->ta_envp[info->ta_envc];
      info->ta_envp[info->ta_envc] = NULL;
    }

  /* Free the old environment (if there was one) */

  if (info->ta_envc == 0)
    {
      lib_ufree(info->ta_envp);
      info->ta_envp = NULL;
      info->ta_envpc = 0;
    }
  else if (info->ta_envc <=
           (info->ta_envpc - SCHED_ENVIRON_RESERVED * 2))
    {
      /* Reallocate the environment to reclaim a little memory */

      info->ta_envpc = info->ta_envc + SCHED_ENVIRON_RESERVED + 1;

      info->ta_envp = lib_urealloc(info->ta_envp,
         sizeof(*info->ta_envp) * info->ta_envpc);
      DEBUGASSERT(info->ta_envp != NULL);
    }
}
