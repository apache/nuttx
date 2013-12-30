/****************************************************************************
 * include/nuttx/kthread.h
 * Non-standard, NuttX-specific kernel thread-related declarations.
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 * Pre-Processor Definitions
 ****************************************************************************/

/* One processor family supported by NuttX has a single, fixed hardware stack.
 * That is the 8051 family.  So for that family only, there is a variant form
 * of kernel_thread() that does not take a stack size parameter.  The following
 * helper macro is provided to work around the ugliness of that exception.
 */

#ifndef CONFIG_CUSTOM_STACK
#  define KERNEL_THREAD(n,p,s,e,a)   kernel_thread(n,p,s,e,a)
#else
#  define KERNEL_THREAD(n,p,s,e,a)   kernel_thread(n,p,e,a)
#endif

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

/********************************************************************************
 * Name: kernel_thread
 *
 * Description:
 *   This function creates and activates a kernel thread task with kernel-mode
 *   privileges.  It is identical to task_create() except that it configures the
 *   newly started thread to run in kernel model.
 *
 * Input Parameters:
 *   (same as task_create())
 *
 * Return Value:
 *   (same as task_create())
 *
 ********************************************************************************/

#ifndef CONFIG_CUSTOM_STACK
int kernel_thread(FAR const char *name, int priority, int stack_size,
                  main_t entry, FAR char * const argv[]);
#else
int kernel_thread(FAR const char *name, int priority, main_t entry,
                  FAR char * const argv[]);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_KTHREAD_H */
