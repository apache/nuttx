/****************************************************************************
 * sched/environ/env_removevar.c
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

#include <string.h>
#include <sched.h>
#include <assert.h>

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
 *   group - The task group with the environment containing the name=value
 *           pair
 *   pvar  - A pointer to the name=value pair in the restroom
 *
 * Returned Value:
 *   Zero on success
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Caller has pre-emption disabled
 *   - Caller will reallocate the environment structure to the correct size
 *
 ****************************************************************************/

int env_removevar(FAR struct task_group_s *group, FAR char *pvar)
{
  FAR char *end;    /* Pointer to the end+1 of the environment */
  int alloc;        /* Size of the allocated environment */
  int ret = ERROR;

  DEBUGASSERT(group != NULL && pvar != NULL);

  /* Verify that the pointer lies within the environment region */

  alloc = group->tg_envsize;             /* Size of the allocated environment */
  end   = &group->tg_envp[alloc];        /* Pointer to the end+1 of the environment */

  if (pvar >= group->tg_envp && pvar < end)
    {
      /* Set up for the removal */

      int len        = strlen(pvar) + 1; /* Length of name=value string to remove */
      FAR char *src  = &pvar[len];       /* Address of name=value string after */
      FAR char *dest = pvar;             /* Location to move the next string */
      int count      = end - src;        /* Number of bytes to move (might be zero) */

      /* Move all of the environment strings after the removed one 'down.'
       * this is inefficient, but robably not high duty.
       */

      while (count-- > 0)
        {
          *dest++ = *src++;
        }

      /* Then set to the new allocation size.  The caller is expected to
       * call realloc at some point but we don't do that here because the
       * caller may add more stuff to the environment.
       */

      group->tg_envsize -= len;
      ret = OK;
    }

  return ret;
}

#endif /* CONFIG_DISABLE_ENVIRON */
