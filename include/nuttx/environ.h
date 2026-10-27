/****************************************************************************
 * include/nuttx/environ.h
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

#ifndef __INCLUDE_NUTTX_ENVIRON_H
#define __INCLUDE_NUTTX_ENVIRON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#ifdef CONFIG_DISABLE_ENVIRON
#  define env_dup(info, envp) (0)
#  define env_release(info)   ((void)0)
#else

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Callback function used with env_foreach() */

typedef CODE int (*env_foreach_t)(FAR void *arg, FAR const char *pair);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct task_info_s;  /* Forward reference */

/****************************************************************************
 * Name: env_foreach
 *
 * Description:
 *   Search the provided environment structure for the variable of the
 *   specified name.
 *
 *   This is an internal OS function and should not be used by applications.
 *
 * Input Parameters:
 *   group - The task group containing environment array to be searched.
 *   cb    - The callback function to be invoked for each environment
 *           variable.
 *
 * Returned Value:
 *   Zero if the all environment variables have been traversed.  A non-zero
 *   value means that the callback function requested early termination by
 *   returning a nonzero value.
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emption is disabled by caller
 *
 ****************************************************************************/

int env_foreach(FAR struct task_info_s *info, env_foreach_t cb,
                FAR void *arg);

/****************************************************************************
 * Name: env_dup
 *
 * Description:
 *   Copy the internal environment structure of a task.  This is the action
 *   that is performed when a new task is created:
 *   The new task has a private, exact duplicate of the parent task's
 *    environment.
 *
 * Input Parameters:
 *   info  - The child task to receive the newly allocated copy of the
 *           parent task groups environment structure.
 *   envp  - Pointer to the environment strings.
 *
 * Returned Value:
 *   zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

int env_dup(FAR struct task_info_s *info, FAR char * const *envp);

/****************************************************************************
 * Name: env_release
 *
 * Description:
 *   env_release() is called only from group_leave() when the last member of
 *   a task group exits.  The env_release() function clears the environment
 *   of all name-value pairs and sets the value of the external variable
 *   environ to NULL.
 *
 * Input Parameters:
 *   info - Identifies the task containing the environment structure
 *          to be released.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

void env_release(FAR struct task_info_s *info);

#ifdef CONFIG_SCHED_USER_IDENTITY
/****************************************************************************
 * Name: env_sanitize_secure
 *
 * Description:
 *   Remove environment variables that must not be inherited by a secure
 *   (set-user-ID or set-group-ID) executable.
 *
 ****************************************************************************/

void env_sanitize_secure(FAR struct task_info_s *info);
#endif

#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_DISABLE_ENVIRON */
#endif /* __INCLUDE_NUTTX_ENVIRON_H */
