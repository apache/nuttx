/****************************************************************************
 * arch/arm64/src/common/arm64_fork_func.S
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
 * Name: fork
 *
 * Description:
 *   The up_fork() function is the base of fork() function that provided in
 *   libc, and fork() is implemented as a wrapper of up_fork() function.
 *   The fork() function has the same effect as posix fork(), except that the
 *   behavior is undefined if the process created by fork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from fork(), or returns from the function in which fork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This thin layer implements fork by simply calling up_fork() with the
 *   fork() context as an argument.  The overall sequence is:
 *
 *   1) User code calls fork().  fork() collects context information and
 *      transfers control up up_fork().
 *   2) arm64_fork() and calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) arm64_fork() provides any additional operating context. arm64_fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) arm64_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
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

    /* Then, call arm64_fork(), passing it a pointer to the stack structure */

    mov    x0, sp
    mov    x1, #0
    bl  arm64_fork

    /* Release the stack data and return the value returned by arm64_fork */

    ldp    x29, x30, [sp, #8 * FORK_REG_FP]
    add    sp, sp,  #8 * FORK_REGS_SIZE

    ret
