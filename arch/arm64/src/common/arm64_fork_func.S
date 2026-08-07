/****************************************************************************
 * arch/arm64/src/common/arm64_fork_func.S
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

#include "arch/syscall.h"
#include "arm64_macro.inc"
#include "arm64_fork.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm64_fork_func.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fork
 *
 * Description:
 *   The architecture-specific entry point of both of NuttX's cloning
 *   primitives.  Both need exactly the same thing from assembly -- a
 *   snapshot of the caller's registers, stack pointer and return address --
 *   and differ only in what the C code then does with it, so there is one
 *   entry point and the caller's x0 says which primitive was called.  It is
 *   saved into the snapshot along with the other argument registers, which
 *   is harmless:  x0-x18 are caller-saved and arm64_fork() does not
 *   propagate them to the child.
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork() or fork().  Both are libc wrappers around
 *      this entry point.
 *   2) The entry point collects the context and calls arm64_fork().
 *   3) arm64_fork() calls nxtask_setup_fork(), which allocates and
 *      configures the child task's TCB.  This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Establishing the child's address environment
 *      - Allocating the stack, or inheriting the parent's for fork()
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) arm64_fork() provides any additional operating context:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *      - Relocate the copied stack, unless the child shares the parent's
 *   5) arm64_fork() then calls nxtask_start_fork()
 *   6) which executes the child thread.
 *
 * Input Parameters:
 *   x0 - true for vfork(), false for fork()
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned to the child and the process
 *   ID of the child is returned to the parent.  Otherwise, -1 is returned to
 *   the parent, no child is created, and errno is set to indicate the error.
 *
 ****************************************************************************/

GTEXT(up_fork)
SECTION_FUNC(text, up_fork)
    /* Create a stack frame */

    sub    sp,  sp, #8 * FORK_REGS_SIZE    /* Allocate the structure on the stack */
    stp    x29, x30, [sp, #8 * FORK_REG_FP] /* Save the FP, LR */

    /* CPU registers, save all register*/

    stp    x0, x1, [sp, #8 * FORK_REG_X0]
    stp    x2, x3, [sp, #8 * FORK_REG_X2]
    stp    x4, x5, [sp, #8 * FORK_REG_X4]
    stp    x6, x7, [sp, #8 * FORK_REG_X6]
    stp    x8, x9, [sp, #8 * FORK_REG_X8]
    stp    x10, x11, [sp, #8 * FORK_REG_X10]
    stp    x12, x13, [sp, #8 * FORK_REG_X12]
    stp    x14, x15, [sp, #8 * FORK_REG_X14]
    stp    x16, x17, [sp, #8 * FORK_REG_X16]
    stp    x18, x19, [sp, #8 * FORK_REG_X18]
    stp    x20, x21, [sp, #8 * FORK_REG_X20]
    stp    x22, x23, [sp, #8 * FORK_REG_X22]
    stp    x24, x25, [sp, #8 * FORK_REG_X24]
    stp    x26, x27, [sp, #8 * FORK_REG_X26]
    str    x28,      [sp, #8 * FORK_REG_X28]

    /* Save the stack pointer */

    add    x0, sp, #8 * FORK_REGS_SIZE
    str    x0, [sp, #8 * FORK_REG_SP]

    /* Floating point registers */
#ifdef CONFIG_ARCH_FPU
    mov     x0, sp
    stp  x0, x30, [sp, #-16]!
    bl     arm64_fork_fpureg_save
    ldp  x0, x30, [sp], #16
#endif

    /* Then, call arm64_fork(), passing it the vfork flag and a pointer to
     * the stack structure.  The flag comes back out of the snapshot:  the
     * sequence above clobbers x0, but it saved it first.
     */

    ldr    x0, [sp, #8 * FORK_REG_X0]
    mov    x1, sp
    bl  arm64_fork

    /* Release the stack data and return the value returned by arm64_fork */

    ldp    x29, x30, [sp, #8 * FORK_REG_FP]
    add    sp, sp,  #8 * FORK_REGS_SIZE

    ret
