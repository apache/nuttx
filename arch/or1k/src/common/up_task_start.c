/****************************************************************************
 * arch/or1k/src/common/up_task_start.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include "svcall.h"
#include "up_internal.h"

#if defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_task_start
 *
 * Description:
 *   In this kernel mode build, this function will be called to execute a
 *   task in user-space.  When the task is first started, a kernel-mode
 *   stub will first run to perform some housekeeping functions.  This
 *   kernel-mode stub will then be called transfer control to the user-mode
 *   task.
 *
 *   Normally the a user-mode start-up stub will also execute before the
 *   task actually starts.  See libc/sched/task_startup.c
 *
 * Input Parameters:
 *   taskentry - The user-space entry point of the task.
 *   argc - The number of parameters being passed.
 *   argv - The parameters being passed. These lie in kernel-space memory
 *     and will have to be reallocated  in user-space memory.
 *
 * Returned Value:
 *   This function should not return.  It should call the user-mode start-up
 *   stub and that stub should call exit if/when the user task terminates.
 *
 ****************************************************************************/

void up_task_start(main_t taskentry, int argc, FAR char *argv[])
{
  /* Let sys_call3() do all of the work */

  sinfo("entry %p argc %d\n", taskentry, argc);

  sys_call3(SYS_task_start, (uintptr_t)taskentry, (uintptr_t)argc,
            (uintptr_t)argv);
}

#endif /* CONFIG_BUILD_PROTECTED || CONFIG_BUILD_KERNEL */
