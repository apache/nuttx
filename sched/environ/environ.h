/****************************************************************************
 * sched/environ/environ.h
 *
 *   Copyright (C) 2007, 2009, 2013-2014, 2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#  define env_dup(group)     (0)
#  define env_release(group) (0)
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
 *   that is performed when a new task is created: The new task has a private,
 *   exact duplicate of the parent task's environment.
 *
 * Input Parameters:
 *   group - The child task group to receive the newly allocated copy of the
 *           parent task groups environment structure.
 *
 * Returned Value:
 *   zero on success
 *
 * Assumptions:
 *   Not called from an interrupt handler.
 *
 ****************************************************************************/

int env_dup(FAR struct task_group_s *group);

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
 *   A pointer to the name=value string in the environment
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emption is disabled by caller
 *
 ****************************************************************************/

FAR char *env_findvar(FAR struct task_group_s *group, FAR const char *pname);

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

int env_removevar(FAR struct task_group_s *group, FAR char *pvar);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_DISABLE_ENVIRON */
#endif /* __SCHED_ENVIRON_ENVIRON_H */
