/****************************************************************************
 * arch/x86_64/src/common/fork.S
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

#include "x86_64_fork.h"
#include "x86_64_internal.h"

	.file	"fork.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fork
 *
 * Description:
 *   The architecture-specific entry point of both of NuttX's cloning
 *   primitives, vfork() and fork().  Both need exactly the same thing from
 *   assembly -- a snapshot of the caller's registers, stack pointer and
 *   return address -- and differ only in what the C code then does with it,
 *   so there is one entry point and the caller's %rdi says which primitive
 *   was called.  It is passed straight through to x86_64_fork().
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork() or fork().  Both are libc wrappers around
 *      this entry point, which collects context information and
 *   2) calls x86_64_fork(), which calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) x86_64_fork() provides any additional operating context. arm_fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) x86_64_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * Input Parameters:
 *   %rdi - true for vfork(), false for fork()
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

/* Stack Layout:
 * | ......... |
 * | rip       | <- rsp when enter up_fork
 * | [ss]      |
 * | [rflags]  |
 * | [cs]      |
 * | [rsp]     | S
 * | [rbp]     | a
 * | [rbx]     | v
 * | [r15]     | i
 * | [r14]     | n
 * | [r13]     | g
 * | [r12]     | <- rsp before calling x86_64_fork, rsi = rsp
 * | ......... |
 */

    .globl  up_fork
    .type   up_fork, @function

up_fork:
    /* %rdi still holds the vfork flag on entry and is left alone;  %rdx is
     * the scratch used to push %ss and %cs.
     */

    movq    %rsp, %rax
    addq    $8, %rax
    movq    %ss, %rdx
    pushq   %rdx
    pushfq
    movq    %cs, %rdx
    pushq   %rdx

    /* push %rsp */

    pushq   %rax
    pushq   %rbp
    pushq   %rbx
    pushq   %r15
    pushq   %r14
    pushq   %r13
    pushq   %r12
    movq    %rsp, %rsi

    subq    $8, %rsp

    /* call function */

    callq x86_64_fork

    addq    $8, %rsp

    /* Do not modify return value %rax */
    /* Restore non-volatile registers */

    popq   %r12
    popq   %r13
    popq   %r14
    popq   %r15
    popq   %rbx
    popq   %rbp
    /* consume %rsp */
    popq   %rdi
    popq   %rdi
    popq   %rdi
    popq   %rdi
    retq
