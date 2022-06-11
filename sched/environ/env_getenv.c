/****************************************************************************
 * sched/environ/env_getenv.c
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

#include <sched.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "sched/sched.h"
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
 *   The value of the valiable (read-only) or NULL on failure
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

FAR char *getenv(FAR const char *name)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *group;
  FAR char *pvalue = NULL;
  int ret = OK;

  /* Verify that a string was passed */

  if (name == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get a reference to the thread-private environ in the TCB. */

  sched_lock();
  rtcb  = this_task();
  group = rtcb->group;

  /* Check if the variable exists */

  if (group == NULL || (ret = env_findvar(group, name)) < 0)
    {
      goto errout_with_lock;
    }

  /* It does!  Get the value sub-string from the name=value string */

  pvalue = strchr(group->tg_envp[ret], '=');
  if (pvalue == NULL)
    {
      /* The name=value string has no '='  This is a bug! */

      ret = -EINVAL;
      goto errout_with_lock;
    }

  /* Adjust the pointer so that it points to the value right after the '=' */

  pvalue++;
  sched_unlock();
  return pvalue;

errout_with_lock:
  sched_unlock();
errout:
  set_errno(-ret);
  return NULL;
}

#endif /* CONFIG_DISABLE_ENVIRON */
