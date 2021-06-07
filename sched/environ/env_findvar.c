/****************************************************************************
 * sched/environ/env_findvar.c
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

#include <stdbool.h>
#include <string.h>
#include <sched.h>
#include <assert.h>

#include "environ/environ.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_cmpname
 ****************************************************************************/

static bool env_cmpname(const char *pszname, const char *peqname)
{
  /* Search until we find anything different in the two names */

  for (; *pszname == *peqname; pszname++, peqname++)
    {
    }

  /* On success, pszname will end with '\0' and peqname with '=' */

  if (*pszname == '\0' && *peqname == '=')
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_findvar
 *
 * Description:
 *   Search the provided environment structure for the variable of the
 *   specified name.
 *
 * Input Parameters:
 *   group - The task group containing environment array to be searched.
 *   pname - The variable name to find
 *
 * Returned Value:
 *   A pointer to the name=value string in the environment
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emption is disabled by caller
 *
 ****************************************************************************/

FAR char *env_findvar(FAR struct task_group_s *group, FAR const char *pname)
{
  FAR char *ptr;
  FAR char *end;

  /* Verify input parameters */

  DEBUGASSERT(group != NULL && pname != NULL);

  /* Search for a name=value string with matching name */

  end = &group->tg_envp[group->tg_envsize];
  for (ptr = group->tg_envp;
       ptr < end && !env_cmpname(pname, ptr);
       ptr += (strlen(ptr) + 1));

  /* Check for success */

  return (ptr < end) ? ptr : NULL;
}

#endif /* CONFIG_DISABLE_ENVIRON */
