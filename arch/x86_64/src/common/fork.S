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
 *   2) x86_64_fork() and calls nxtask_setup_fork().
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
 *   None
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
 * | [r12]     | <- rsp before calling x86_64_fork, rdi = rsp
 * | ......... |
 */

    .globl  up_fork
    .type   up_fork, @function

up_fork:
    movq    %rsp, %rax
    addq    $8, %rax
    movq    %ss, %rdi
    pushq   %rdi
    pushfq
    movq    %cs, %rdi
    pushq   %rdi

    /* push %rsp */

    pushq   %rax
    pushq   %rbp
    pushq   %rbx
    pushq   %r15
    pushq   %r14
    pushq   %r13
    pushq   %r12
    movq    %rsp, %rdi

    /* call function */

    callq x86_64_fork

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
