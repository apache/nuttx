/****************************************************************************
 * arch/arm/src/common/vfork.S
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

	.syntax	unified
	.file	"vfork.S"

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

	.globl	vfork
	.type	vfork, function

vfork:
	/* Create a stack frame */

	mov		r0, sp			/* Save the value of the stack on entry */
	sub		sp, sp, #VFORK_SIZEOF	/* Allocate the structure on the stack */

	/* CPU registers */
	/* Save the volatile registers */

	mov		r1, sp
	stmia		r1!, {r4-r7}		/* Save r4-r7 in the structure */
	mov		r4, r8			/* Copy high registers to low registers */
	mov		r5, r9
	mov		r6, r10
	mov		r7, fp
	stmia		r1!, {r4-r7}		/* Save r8-r10 and fp in the structure */
	mov		r5, lr			/* Copy lr to a low register */
	stmia		r1!, {r0,r5}		/* Save sp and lr in the structure */

	/* Then, call up_vfork(), passing it a pointer to the stack structure */

	mov		r0, sp
	bl		up_vfork

	/* Recover r4-r7 that were destroyed before up_vfork was called */

	mov		r1, sp
	ldmia		r1!, {r4-r7}

	/* Release the stack data and return the value returned by up_vfork */

	ldr		r1, [sp, #VFORK_LR_OFFSET]
	mov		r14, r1
	add		sp, sp, #VFORK_SIZEOF
	bx		lr

	.size	vfork, .-vfork
	.end
