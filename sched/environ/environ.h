/****************************************************************************
 * sched/environ/environ.h
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

#ifndef __SCHED_ENVIRON_ENVIRON_H
#define __SCHED_ENVIRON_ENVIRON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DISABLE_ENVIRON
#  define env_dup(group, envp) (0)
#  define env_release(group)   (0)
#else

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
 *   group - The child task group to receive the newly allocated copy of the
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

int env_dup(FAR struct task_group_s *group, FAR char * const *envp);

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
 *   group - Identifies the task group containing the environment structure
 *           to be released.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Not called from an interrupt handler
 *
 ****************************************************************************/

void env_release(FAR struct task_group_s *group);

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
 *   A index to the name=value string in the environment
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emption is disabled by caller
 *
 ****************************************************************************/

ssize_t env_findvar(FAR struct task_group_s *group, FAR const char *pname);

/****************************************************************************
 * Name: env_removevar
 *
 * Description:
 *   Remove the referenced name=value pair from the environment
 *
 * Input Parameters:
 *   group - The task group with the environment containing the name=value
 *           pair
 *   index - A index to the name=value pair in the restroom
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Caller has pre-emption disabled
 *   - Caller will reallocate the environment structure to the correct size
 *
 ****************************************************************************/

void env_removevar(FAR struct task_group_s *group, ssize_t index);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_DISABLE_ENVIRON */
#endif /* __SCHED_ENVIRON_ENVIRON_H */
