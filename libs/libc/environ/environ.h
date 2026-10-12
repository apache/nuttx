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

#ifdef __KERNEL__
#  include <nuttx/kmalloc.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON

#  define SCHED_ENVIRON_RESERVED (4)

/* The environment is in the heap that holds the task information: the
 * kernel heap for a kernel thread, the user heap for a user task.  Kernel
 * code also changes the environment of kernel threads (the idle task at
 * boot) and of a new user task.
 */

#  ifdef __KERNEL__
#    define env_malloc(info, n) \
       (kmm_heapmember(info) ? kmm_malloc(n) : kumm_malloc(n))
#    define env_realloc(info, p, n) \
       (kmm_heapmember(info) ? kmm_realloc(p, n) : kumm_realloc(p, n))
#    define env_free(info, p) \
       (kmm_heapmember(info) ? kmm_free(p) : kumm_free(p))
#  else
#    define env_malloc(info, n)     ((void)(info), lib_umalloc(n))
#    define env_realloc(info, p, n) ((void)(info), lib_urealloc(p, n))
#    define env_free(info, p)       ((void)(info), lib_ufree(p))
#  endif

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
 * Name: env_findvar
 *
 * Description:
 *   Search the provided environment structure for the variable of the
 *   specified name.
 *
 * Input Parameters:
 *   info  - The task containing environment array to be searched.
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

ssize_t env_findvar(FAR struct task_info_s *info, FAR const char *pname);

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
 *   - Caller has pre-emption disabled
 *   - Caller will reallocate the environment structure to the correct size
 *
 ****************************************************************************/

void env_removevar(FAR struct task_info_s *info, ssize_t index);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* !CONFIG_DISABLE_ENVIRON */
#endif /* __LIBS_LIBC_ENVIRON_ENVIRON_H */
