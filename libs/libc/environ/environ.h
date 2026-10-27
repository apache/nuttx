/****************************************************************************
 * libs/libc/environ/environ.h
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

#ifndef __LIBS_LIBC_ENVIRON_ENVIRON_H
#define __LIBS_LIBC_ENVIRON_ENVIRON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/lib/lib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON

#  define SCHED_ENVIRON_RESERVED (4)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: env_findvar
 *
 * Description:
 *   Search the provided environment structure for the variable of the
 *   specified name.
 *
 * Input Parameters:
 *   info    - The task containing environment array to be searched.
 *   pname   - The variable name to find.  It needs no terminating NUL.
 *   namelen - The length of the name.
 *
 * Returned Value:
 *   A index to the name=value string in the environment
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - Pre-emption is disabled by caller
 *
 ****************************************************************************/

ssize_t env_findvar(FAR struct task_info_s *info, FAR const char *pname,
                    size_t namelen);

/****************************************************************************
 * Name: env_setvar
 *
 * Description:
 *   setenv() with the length of the name, so that putenv() can pass the
 *   name part of its string without a copy.  An empty value removes the
 *   variable, as in setenv().
 *
 * Input Parameters:
 *   name      - The name of the variable.  It needs no terminating NUL.
 *   namelen   - The length of the name.
 *   value     - The new value of the variable.
 *   overwrite - Replace any existing value if non-zero.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int env_setvar(FAR const char *name, size_t namelen,
               FAR const char *value, int overwrite);

/****************************************************************************
 * Name: env_alloc
 *
 * Description:
 *   Allocate a block for the environment of a task.  The block goes on a
 *   list of the task, and env_release() frees it when the task exits.
 *   libc does not free the strings before then, so a pointer that getenv()
 *   returned stays valid, and the program can keep a string or an array
 *   that it took from environ.
 *
 * Input Parameters:
 *   info - The task that owns the environment.
 *   size - The size of the block.
 *
 * Returned Value:
 *   The block, or NULL if there is no memory.
 *
 ****************************************************************************/

FAR void *env_alloc(FAR struct task_info_s *info, size_t size);

/****************************************************************************
 * Name: env_free
 *
 * Description:
 *   Free a block from env_alloc().  Before the task exits, only for a block
 *   that nothing else can reach.
 *
 ****************************************************************************/

void env_free(FAR struct task_info_s *info, FAR void *mem);

/****************************************************************************
 * Name: env_reserve
 *
 * Description:
 *   Make environ an array that libc allocated, with room for more entries.
 *   The program can assign environ, as POSIX allows.  Then the array is
 *   copied, and the array of the program is not changed.  The strings are
 *   not copied.
 *
 * Input Parameters:
 *   info - The task whose environment is changed.
 *   more - The number of entries to add.
 *
 * Returned Value:
 *   Zero (OK) on success; -ENOMEM if there is no memory.
 *
 * Assumptions:
 *   The caller holds ta_lock.
 *
 ****************************************************************************/

int env_reserve(FAR struct task_info_s *info, size_t more);

/****************************************************************************
 * Name: env_removevar
 *
 * Description:
 *   Remove the referenced name=value pair from the environment
 *
 * Input Parameters:
 *   info  - The task with the environment containing the name=value
 *           pair
 *   index - A index to the name=value pair in the restroom
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Not called from an interrupt handler
 *   - The caller holds ta_lock and called env_reserve()
 *
 ****************************************************************************/

void env_removevar(FAR struct task_info_s *info, size_t index);

#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_DISABLE_ENVIRON */
#endif /* __LIBS_LIBC_ENVIRON_ENVIRON_H */
