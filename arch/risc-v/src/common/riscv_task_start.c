/****************************************************************************
 * arch/risc-v/src/common/riscv_task_start.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <assert.h>

#include <arch/syscall.h>

#include "riscv_internal.h"

#include "sched/sched.h"

#ifndef CONFIG_BUILD_FLAT

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

void up_task_start(main_t taskentry, int argc, char *argv[])
{
  struct tcb_s *rtcb = this_task();
  uintptr_t sp;

#ifdef CONFIG_ARCH_KERNEL_STACK
  sp = (uintreg_t)rtcb->xcp.ustkptr;
#else
  sp = (uintptr_t)rtcb->stack_base_ptr + rtcb->adj_stack_size;
#endif

  /* Set up to return to the user-space _start function in
   * unprivileged mode.  We need:
   *
   *   A0    = argc
   *   A1    = argv
   *   EPC   = taskentry
   *   M/SPP = user mode
   */

#if defined(CONFIG_BUILD_PROTECTED)
  riscv_jump_to_user((uintptr_t)USERSPACE->task_startup,
                     (uintptr_t)taskentry, (uintreg_t)argc, (uintreg_t)argv,
                     sp, rtcb->xcp.initregs);
#else
  riscv_jump_to_user((uintptr_t)taskentry, (uintreg_t)argc, (uintreg_t)argv,
                     0, sp, rtcb->xcp.initregs);
#endif
}

#endif /* !CONFIG_BUILD_FLAT */
