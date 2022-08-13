/****************************************************************************
 * arch/arm64/src/common/arm64_vfork_func.S
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
#include "arm64_vfork.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm64_vfork_func.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by vfork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from vfork(), or returns from the function in which vfork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This thin layer implements vfork by simply calling up_vfork() with the
 *   vfork() context as an argument.  The overall sequence is:
 *
 *   1) User code calls vfork().  vfork() collects context information and
 *      transfers control up up_vfork().
 *   2) up_vfork() and calls nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

GTEXT(vfork)
SECTION_FUNC(text, vfork)
    /* Create a stack frame */

    mov    x0,  sp            /* Save the value of the stack on entry */
    stp    x29, x30, [sp]
    sub    sp,  sp, #8 * VFORK_REGS_SIZE    /* Allocate the structure on the stack */

    /* CPU registers, save all register*/

    stp    x0, x1, [sp, #8 * VFORK_REG_X0]
    stp    x2, x3, [sp, #8 * VFORK_REG_X2]
    stp    x4, x5, [sp, #8 * VFORK_REG_X4]
    stp    x6, x7, [sp, #8 * VFORK_REG_X6]
    stp    x8, x9, [sp, #8 * VFORK_REG_X8]
    stp    x10, x11, [sp, #8 * VFORK_REG_X10]
    stp    x12, x13, [sp, #8 * VFORK_REG_X12]
    stp    x14, x15, [sp, #8 * VFORK_REG_X14]
    stp    x16, x17, [sp, #8 * VFORK_REG_X16]
    stp    x18, x19, [sp, #8 * VFORK_REG_X18]
    stp    x20, x21, [sp, #8 * VFORK_REG_X20]
    stp    x22, x23, [sp, #8 * VFORK_REG_X22]
    stp    x24, x25, [sp, #8 * VFORK_REG_X24]
    stp    x26, x27, [sp, #8 * VFORK_REG_X26]
    stp    x28, x29, [sp, #8 * VFORK_REG_X28]

    /* Save the LR, stack pointer */

    stp    x30, x0, [sp, #8 * VFORK_REG_LR]

    /* Floating point registers */
#ifdef CONFIG_ARCH_FPU
    mov     x0, sp
    stp  x0, x30, [sp, #-16]!
    bl     arm64_vfork_fpureg_save
    ldp  x0, x30, [sp], #16
#endif

    /* Then, call up_vfork(), passing it a pointer to the stack structure */

    mov    x0, sp
    bl  up_vfork

    /* Release the stack data and return the value returned by up_vfork */

    add    sp, sp,  #8 * VFORK_REGS_SIZE
    ldp    x29, x30, [sp]

    ret
