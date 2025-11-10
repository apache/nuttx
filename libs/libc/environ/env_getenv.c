/****************************************************************************
 * libs/libc/environ/env_getenv.c
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
 * Name: getenv
 *
 * Description:
 *   The getenv() function searches the environment list for a string that
 *   matches the string pointed to by name.
 *
 * Input Parameters:
 *   name - The name of the variable to find.
 *
 * Returned Value:
 *   The value of the variable (read-only) or NULL on failure
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

FAR char *getenv(FAR const char *name)
{
  FAR struct task_info_s *info = task_get_info();
  FAR char *pvalue = NULL;
  ssize_t ret = OK;

  /* Verify that a string was passed */

  if (name == NULL || info == NULL
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
      || up_interrupt_context() || sched_idletask()
#endif
     )
    {
      ret = -EINVAL;
      goto errout;
    }

  nxrmutex_lock(&info->ta_lock);
  ret = env_findvar(info, name);
  if (ret < 0)
    {
      nxrmutex_unlock(&info->ta_lock);
      goto errout;
    }

  /* It does!  Get the value sub-string from the name=value string */

  pvalue = strchr(info->ta_envp[ret], '=');
  if (pvalue == NULL)
    {
      /* The name=value string has no '='  This is a bug! */

      nxrmutex_unlock(&info->ta_lock);
      ret = -EINVAL;
      goto errout;
    }

  /* Adjust the pointer so that it points to the value right after the '=' */

  pvalue++;
  nxrmutex_unlock(&info->ta_lock);
  return pvalue;

errout:
  set_errno(-ret);
  return NULL;
}
