/****************************************************************************
 * arch/arm/src/common/iar/fork.S
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

#include "arm_fork.h"

	MODULE up_fork
	SECTION .text:CODE:NOROOT(2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	PUBLIC	up_fork
	EXTERN	arm_fork

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

	THUMB

up_fork:
	/* Create a stack frame */

	mov		r0, sp			/* Save the value of the stack on entry */
	sub		sp, sp, #FORK_SIZEOF	/* Allocate the structure on the stack */

	/* CPU registers */
	/* Save the volatile registers */

	str		r4, [sp, #FORK_R4_OFFSET]
	str		r5, [sp, #FORK_R5_OFFSET]
	str		r6, [sp, #FORK_R6_OFFSET]
	str		r7, [sp, #FORK_R7_OFFSET]
	str		r8, [sp, #FORK_R8_OFFSET]
	str		r9, [sp, #FORK_R9_OFFSET]
	str		r10, [sp, #FORK_R10_OFFSET]

	/* Save the frame pointer, stack pointer, and return address */

	str		r11, [sp, #FORK_FP_OFFSET] /* fp not defined. use r11 */
	str		r0, [sp, #FORK_SP_OFFSET]
	str		lr, [sp, #FORK_LR_OFFSET]

	/* Floating point registers (not yet) */

	/* Then, call arm_fork(), passing it a pointer to the stack structure */

	mov		r0, sp
	bl		arm_fork

	/* Release the stack data and return the value returned by arm_fork */

	ldr		lr, [sp, #FORK_LR_OFFSET]
	add		sp, sp, #FORK_SIZEOF
	bx		lr

	END
