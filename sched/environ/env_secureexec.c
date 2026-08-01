/****************************************************************************
 * sched/environ/env_secureexec.c
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

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "sched/sched.h"
#include "environ/environ.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_USER_IDENTITY

/****************************************************************************
 * Name: env_is_unsafe
 *
 * Description:
 *   Return true if the "NAME=value" pair must not be inherited by a secure
 *   (set-user-ID or set-group-ID) executable.
 *
 ****************************************************************************/

static bool env_is_unsafe(FAR const char *pair)
{
  static FAR const char * const names[] =
    {
      "IFS=",
      "CDPATH=",
      "ENV=",
      "BASH_ENV=",
      "PATH=",
      "HOSTALIASES=",
      "LOCPATH=",
      "GCONV_PATH=",
      "TZDIR=",
      "LOCALDOMAIN=",
      "NLSPATH=",
      NULL
    };

  FAR const char * const *name;

  for (name = names; *name != NULL; name++)
    {
      if (strncmp(pair, *name, strlen(*name)) == 0)
        {
          return true;
        }
    }

  return strncmp(pair, "LD_", 3) == 0 ||
         strncmp(pair, "MALLOC_", 7) == 0;
}

#endif /* CONFIG_SCHED_USER_IDENTITY */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_sanitize_secure
 ****************************************************************************/

#ifdef CONFIG_SCHED_USER_IDENTITY
void env_sanitize_secure(FAR struct task_group_s *group)
{
  ssize_t index;

  DEBUGASSERT(group != NULL);

  nxrmutex_lock(&group->tg_mutex);

  for (index = group->tg_envc - 1; index >= 0; )
    {
      if (env_is_unsafe(group->tg_envp[index]))
        {
          env_removevar(group, index);
        }
      else
        {
          index--;
        }
    }

  nxrmutex_unlock(&group->tg_mutex);
}
#endif

/****************************************************************************
 * Name: secure_getenv
 ****************************************************************************/

FAR char *secure_getenv(FAR const char *name)
{
  return issetugid() ? NULL : getenv(name);
}
