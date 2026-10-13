/****************************************************************************
 * libs/libc/environ/env_unsetenv.c
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

#include <errno.h>
#include <string.h>

#include <nuttx/mutex.h>
#include <nuttx/tls.h>

#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unsetenv
 *
 * Description:
 *   The unsetenv() function deletes the variable name from the environment.
 *
 * Input Parameters:
 *   name - The name of the variable to delete
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

int unsetenv(FAR const char *name)
{
  FAR struct task_info_s *info = task_get_info();
  ssize_t idx;
  int ret;

  /* Check the incoming parameter */

  if (name == NULL || *name == '\0' || strchr(name, '=') != NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  nxrmutex_lock(&info->ta_lock);

  /* Check if the variable exists */

  if ((idx = env_findvar(info, name, strlen(name))) >= 0)
    {
      /* It does!  Remove the name=value pair from an array of libc. */

      ret = env_reserve(info, 0);
      if (ret < 0)
        {
          nxrmutex_unlock(&info->ta_lock);
          set_errno(-ret);
          return ERROR;
        }

      env_removevar(info, idx);
    }

  nxrmutex_unlock(&info->ta_lock);
  return OK;
}
