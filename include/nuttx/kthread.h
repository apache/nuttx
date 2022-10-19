/****************************************************************************
 * include/nuttx/kthread.h
 * Non-standard, NuttX-specific kernel thread-related declarations.
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

#ifndef __INCLUDE_NUTTX_KTHREAD_H
#define __INCLUDE_NUTTX_KTHREAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: kthread_create_with_stack
 *
 * Description:
 *   This function creates and activates a kernel thread task with
 *   kernel-mode privileges. It is identical to kthread_create() except
 *   that it get the stack memory from caller.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_addr - Stack buffer of the new task
 *   stack_size - Stack size of the new task
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

int kthread_create_with_stack(FAR const char *name, int priority,
                              FAR void *stack_addr, int stack_size,
                              main_t entry, FAR char * const argv[]);

/****************************************************************************
 * Name: kthread_create
 *
 * Description:
 *   This function creates and activates a kernel thread task with
 *   kernel-mode privileges. It is identical to task_create() except
 *   that it configures the newly started thread to run in kernel model.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

int kthread_create(FAR const char *name, int priority, int stack_size,
                   main_t entry, FAR char * const argv[]);

/****************************************************************************
 * Name: kthread_delete
 *
 * Description:
 *   This function will terminate a kernel thread.  At present, this
 *   function is equivalent to task_delete.c; the following definition
 *   simply reserves the name in the name space.
 *
 *   Refer to comments with nxtask_delete() for a more detailed description
 *   of the operation of this function.
 *
 * Input Parameters:
 *   pid - The task ID of the task to delete.  A pid of zero
 *         signifies the calling task.
 *
 * Returned Value:
 *   OK on success; or negated errno on failure.
 *
 ****************************************************************************/

#define kthread_delete(p) nxtask_delete(p)

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_KTHREAD_H */
