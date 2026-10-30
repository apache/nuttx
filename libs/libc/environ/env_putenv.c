/****************************************************************************
 * libs/libc/environ/env_putenv.c
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
#include <string.h>
#include <errno.h>

#include <nuttx/mutex.h>
#include <nuttx/tls.h>

#include "environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: putenv
 *
 * Description:
 *   The putenv() function adds or changes the value of environment
 *   variables.
 *   The argument string is of the form name=value. If name does not already
 *   exist in  the  environment, then string is added to the environment. If
 *   name does exist, then the value of name in the environment is changed to
 *   value.
 *
 *   As POSIX requires, the string itself becomes part of the environment:
 *   a later change to the string changes the environment.  libc does not
 *   copy or free it.
 *
 * Input Parameters:
 *   name=value string describing the environment setting to add/modify
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int putenv(FAR char *string)
{
  FAR struct task_info_s *info = task_get_info();
  FAR char *pequal;
  ssize_t idx;
  int ret;

  /* Verify that a string was passed */

  if (!string)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Parse the name=value string */

  pequal = strchr(string, '=');
  if (pequal == NULL)
    {
      return OK;
    }

  DEBUGASSERT(info != NULL);

  nxrmutex_lock(&info->ta_lock);

  /* environ must be an array of libc with room for one more entry */

  ret = env_reserve(info, 1);
  if (ret >= 0)
    {
      /* Put the string itself in the place of the old name=value pair, or
       * at the end.
       */

      idx = env_findvar(info, string, pequal - string);
      if (idx >= 0)
        {
          info->ta_envp[idx] = string;
        }
      else
        {
          info->ta_envp[info->ta_envc++] = string;
          info->ta_envp[info->ta_envc]   = NULL;
        }
    }

  nxrmutex_unlock(&info->ta_lock);

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
