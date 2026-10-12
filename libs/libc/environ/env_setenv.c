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
  FAR struct task_info_s *info;
  FAR char *pvar;
  FAR char **envp;
  ssize_t envc;
  ssize_t envpc;
  ssize_t ret = OK;
  int varlen;

  /* Verify input parameter */

  if (!name)
    {
      ret = EINVAL;
      goto errout;
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

  /* Get a reference to the thread-private environ in the TCB. */

  info = task_get_info();

  DEBUGASSERT(info != NULL);

  nxrmutex_lock(&info->ta_lock);

  /* Check if the variable already exists */

  if (info->ta_envp && (ret = env_findvar(info, name)) >= 0)
    {
      /* It does! Do we have permission to overwrite the existing value? */

      if (!overwrite)
        {
          /* No.. then just return success */

          nxrmutex_unlock(&info->ta_lock);
          return OK;
        }

      /* Yes.. just remove the name=value pair from the environment.  It will
       * be added again below.  Note that we are responsible for reallocating
       * the environment buffer; this will happen below.
       */

      env_removevar(info, ret);
    }

  /* Check current envirments count */

  DEBUGASSERT(info->ta_envc < SSIZE_MAX);

  /* Get the size of the new name=value string.
   * The +2 is for the '=' and for null terminator
   */

  varlen = strlen(name) + strlen(value) + 2;

  /* Then allocate or reallocate the environment buffer */

  pvar = env_malloc(info, varlen);
  if (pvar == NULL)
    {
      ret = ENOMEM;
      goto errout_with_lock;
    }

  envc = info->ta_envc;

  if (info->ta_envp == NULL)
    {
      envpc = SCHED_ENVIRON_RESERVED + 2;

      envp = env_malloc(info, sizeof(*envp) * envpc);
      if (envp == NULL)
        {
          ret = ENOMEM;
          goto errout_with_var;
        }

      info->ta_envp  = envp;
      info->ta_envpc = envpc;
    }
  else if (envc >= info->ta_envpc - 1)
    {
      envpc = envc + SCHED_ENVIRON_RESERVED + 2;

      envp = env_realloc(info, info->ta_envp, sizeof(*envp) * envpc);
      if (envp == NULL)
        {
          ret = ENOMEM;
          goto errout_with_var;
        }

      info->ta_envp  = envp;
      info->ta_envpc = envpc;
    }

  /* Save the new buffer and count */

  info->ta_envp[envc++] = pvar;
  info->ta_envp[envc]   = NULL;
  info->ta_envc = envc;

  /* Now, put the new name=value string into the environment buffer */

  snprintf(pvar, varlen, "%s=%s", name, value);
  nxrmutex_unlock(&info->ta_lock);
  return OK;

errout_with_var:
  env_free(info, pvar);
errout_with_lock:
  nxrmutex_unlock(&info->ta_lock);
errout:
  set_errno(ret);
  return ERROR;
}
