/****************************************************************************
 * sched/environ/env_setenv.c
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

#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>

#include "sched/sched.h"
#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setenv
 *
 * Description:
 *   The setenv() function adds the variable name to the environment with the
 *   specified 'value' if the varialbe 'name" does not exist. If the 'name'
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
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *group;
  FAR char *pvar;
  FAR char *newenvp;
  int newsize;
  int varlen;
  int ret = OK;

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

  sched_lock();
  rtcb  = this_task();
  group = rtcb->group;
  DEBUGASSERT(group);

  /* Check if the variable already exists */

  if (group->tg_envp && (pvar = env_findvar(group, name)) != NULL)
    {
      /* It does! Do we have permission to overwrite the existing value? */

      if (!overwrite)
        {
          /* No.. then just return success */

          sched_unlock();
          return OK;
        }

      /* Yes.. just remove the name=value pair from the environment.  It will
       * be added again below.  Note that we are responsible for reallocating
       * the environment buffer; this will happen below.
       */

      env_removevar(group, pvar);
    }

  /* Get the size of the new name=value string.
   * The +2 is for the '=' and for null terminator
   */

  varlen = strlen(name) + strlen(value) + 2;

  /* Then allocate or reallocate the environment buffer */

  if (group->tg_envp)
    {
      newsize = group->tg_envsize + varlen;
      newenvp = (FAR char *)kumm_realloc(group->tg_envp, newsize);
      if (!newenvp)
        {
          ret = ENOMEM;
          goto errout_with_lock;
        }

      pvar = &newenvp[group->tg_envsize];
    }
  else
    {
      newsize = varlen;
      newenvp = (FAR char *)kumm_malloc(varlen);
      if (!newenvp)
        {
          ret = ENOMEM;
          goto errout_with_lock;
        }

      pvar = newenvp;
    }

  /* Save the new buffer and size */

  group->tg_envp    = newenvp;
  group->tg_envsize = newsize;

  /* Now, put the new name=value string into the environment buffer */

  sprintf(pvar, "%s=%s", name, value);
  sched_unlock();
  return OK;

errout_with_lock:
  sched_unlock();
errout:
  set_errno(ret);
  return ERROR;
}

#endif /* CONFIG_DISABLE_ENVIRON */
