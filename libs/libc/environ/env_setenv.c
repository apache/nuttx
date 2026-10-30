/****************************************************************************
 * libs/libc/environ/env_setenv.c
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
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/mutex.h>
#include <nuttx/tls.h>

#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setenv
 *
 * Description:
 *   The setenv() function adds the variable name to the environment with the
 *   specified 'value' if the variable 'name" does not exist. If the 'name'
 *   does exist in the environment, then its value is changed to 'value' if
 *   'overwrite' is non-zero; if 'overwrite' is zero, then the value of name
 *   unaltered.
 *
 * Input Parameters:
 *   name - The name of the variable to change
 *   value - The new value of the variable
 *   overwrite - Replace any existing value if non-zero.
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int setenv(FAR const char *name, FAR const char *value, int overwrite)
{
  FAR struct task_info_s *info = task_get_info();
  FAR char *pvar;
  ssize_t idx;
  size_t namelen;
  size_t varlen;
  int ret;

  /* Verify input parameter */

  if (!name)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* if no value is provided, then this is the same as unsetenv (unless
   * overwrite is false)
   */

  if (!value || *value == '\0')
    {
      /* If overwrite is set then this is the same as unsetenv */

      if (overwrite)
        {
          return unsetenv(name);
        }
      else
        {
          /* Otherwise, it is a request to remove a variable without
           * altering it?
           */

          return OK;
        }
    }

  DEBUGASSERT(info != NULL);

  namelen = strlen(name);
  nxrmutex_lock(&info->ta_lock);

  /* environ must be an array of libc with room for one more entry */

  ret = env_reserve(info, 1);
  if (ret < 0)
    {
      goto out;
    }

  /* Check if the variable already exists */

  idx = env_findvar(info, name, namelen);
  if (idx >= 0)
    {
      /* It does! Do we have permission to overwrite the existing value? */

      if (!overwrite)
        {
          goto out;
        }

      /* Yes.. just remove the name=value pair from the environment.  It will
       * be added again below.
       */

      env_removevar(info, idx);
    }

  /* Get the size of the new name=value string.
   * The +2 is for the '=' and for null terminator
   */

  varlen = namelen + strlen(value) + 2;

  pvar = env_alloc(info, varlen);
  if (pvar == NULL)
    {
      ret = -ENOMEM;
      goto out;
    }

  /* Now, put the new name=value string into the environment */

  snprintf(pvar, varlen, "%s=%s", name, value);

  info->ta_envp[info->ta_envc++] = pvar;
  info->ta_envp[info->ta_envc]   = NULL;

out:
  nxrmutex_unlock(&info->ta_lock);

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
