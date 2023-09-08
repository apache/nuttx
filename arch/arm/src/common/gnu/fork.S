/****************************************************************************
 * arch/arm/src/common/fork.S
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

#include "arm_fork.h"

	.syntax	unified
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
 *   2) arm_fork() and calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) arm_fork() provides any additional operating context. arm_fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) arm_fork() then calls nxtask_start_fork()
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

	.globl	up_fork
	.type	up_fork, function

up_fork:
	/* Create a stack frame */

	mov		r0, sp			/* Save the value of the stack on entry */
	sub		sp, sp, #FORK_SIZEOF	/* Allocate the structure on the stack */

	/* CPU registers */
	/* Save the volatile registers */

	mov		r1, sp
	stmia		r1!, {r4-r7}		/* Save r4-r7 in the structure */
	mov		r4, r8			/* Copy high registers to low registers */
	mov		r5, r9
	mov		r6, r10
	mov		r7, r11
	stmia		r1!, {r4-r7}		/* Save r8-r11 in the structure */
	mov		r5, lr			/* Copy lr to a low register */
	stmia		r1!, {r0,r5}		/* Save sp and lr in the structure */

	/* Then, call arm_fork(), passing it a pointer to the stack structure */

	mov		r0, sp
	bl		arm_fork

	/* Recover r4-r7 that were destroyed before arm_fork was called */

	mov		r1, sp
	ldmia		r1!, {r4-r7}

	/* Release the stack data and return the value returned by up_fork */

	ldr		r1, [sp, #FORK_LR_OFFSET]
	mov		r14, r1
	add		sp, sp, #FORK_SIZEOF
	bx		lr

	.size	up_fork, .-up_fork
	.end
