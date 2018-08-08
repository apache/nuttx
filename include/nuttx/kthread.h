/****************************************************************************
 * include/nuttx/kthread.h
 * Non-standard, NuttX-specific kernel thread-related declarations.
 *
 *   Copyright (C) 2013, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_KTHREAD_H
#define __INCLUDE_NUTTX_KTHREAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sched.h>

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
 * Name: nxtask_create
 *
 * Description:
 *   This function creates and activates a new user task with a specified
 *   priority and returns its system-assigned ID.
 *
 *   The entry address entry is the address of the "main" function of the
 *   task.  This function will be called once the C environment has been
 *   set up.  The specified function will be called with four arguments.
 *   Should the specified routine return, a call to exit() will
 *   automatically be made.
 *
 *   Note that four (and only four) arguments must be passed for the spawned
 *   functions.
 *
 *   nxtask_create() is identical to the function task_create(), differing
 *   only in its return value:  This function does not modify the errno
 *   variable.  This is a non-standard, internal OS function and is not
 *   intended for use by application logic.  Applications should use
 *   task_create().
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

int nxtask_create(FAR const char *name, int priority,
                  int stack_size, main_t entry, FAR char * const argv[]);

/********************************************************************************
 * Name: kthread_create
 *
 * Description:
 *   This function creates and activates a kernel thread task with kernel-mode
 *   privileges.  It is identical to task_create() except that it configures the
 *   newly started thread to run in kernel model.
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
 ********************************************************************************/

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
 *   Refer to comments with task_delete() for a more detailed description of
 *   the operation of this function.
 *
 * Input Parameters:
 *   pid - The task ID of the task to delete.  A pid of zero
 *         signifies the calling task.
 *
 * Returned Value:
 *   OK on success; or ERROR on failure with the errno variable set
 *   appropriately.
 *
 ****************************************************************************/

#define kthread_delete(p) task_delete(p)

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_KTHREAD_H */
