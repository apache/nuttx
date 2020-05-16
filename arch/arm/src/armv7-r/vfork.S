/****************************************************************************
 * arch/arm/src/armv7-r/vfork.S
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

#include "arm_vfork.h"

	.file	"vfork.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	up_vfork

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
 *   2) up_vfork()and calls nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state()
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Allocate and initialize the stack
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

	.globl	vfork
	.type	vfork, function
vfork:
	/* Create a stack frame */

	mov		r0, sp					/* Save the value of the stack on entry */
	sub		sp, sp, #VFORK_SIZEOF	/* Allocate the structure on the stack */

	/* CPU registers */
	/* Save the volatile registers */

	str		r4, [sp, #VFORK_R4_OFFSET]
	str		r5, [sp, #VFORK_R5_OFFSET]
	str		r6, [sp, #VFORK_R6_OFFSET]
	str		r7, [sp, #VFORK_R7_OFFSET]
	str		r8, [sp, #VFORK_R8_OFFSET]
	str		r9, [sp, #VFORK_R9_OFFSET]
	str		r10, [sp, #VFORK_R10_OFFSET]

	/* Save the frame pointer, stack pointer, and return address */

	str		fp, [sp, #VFORK_FP_OFFSET]
	str		r0, [sp, #VFORK_SP_OFFSET]
	str		lr, [sp, #VFORK_LR_OFFSET]

	/* Floating point registers (not yet) */

	/* Then, call up_vfork(), passing it a pointer to the stack structure */

	mov		r0, sp
	bl		up_vfork

	/* Release the stack data and return the value returned by up_vfork */

	ldr		lr, [sp, #VFORK_LR_OFFSET]
	add		sp, sp, #VFORK_SIZEOF
	mov		pc, lr
	.size	vfork, .-vfork
	.end
