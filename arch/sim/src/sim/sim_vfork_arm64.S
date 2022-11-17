/****************************************************************************
 * arch/sim/src/sim/sim_vfork_arm64.S
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
 ***************************************************************************/

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#if defined(CONFIG_HOST_MACOS)
	#define SYMBOL(x) _##x
#else
	#define SYMBOL(x) x
#endif

/****************************************************************************
 * Public Symbols
 ***************************************************************************/

    .file  "up_vfork_arm64.S"
    .globl SYMBOL(up_vfork)
    .globl SYMBOL(setjmp)

/****************************************************************************
 * Public Functions
 ***************************************************************************/

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
 ***************************************************************************/

	.text
	.globl	SYMBOL(vfork)
	.align	4

SYMBOL(vfork):

	stp		x29, x30, [sp]				/* save FP/LR register */
	sub		sp, sp, #XCPTCONTEXT_SIZE	/* area from stack for setjmp() */

	mov		x0, sp						/* pass stack area to setjmp() */
	bl		SYMBOL(setjmp)				/* save register for longjmp() */

	subs	x0, x0, #1					/* 0: parent / 1: child */
	cbz		x0, 1f						/* child --> return */

	mov		x0, sp						/* pass stack area to up_vfork() */
	bl		SYMBOL(up_vfork)			/* further process task creation */

1:
	add		sp, sp, #XCPTCONTEXT_SIZE	/* release area from stack */
	ldp		x29, x30, [sp]				/* restore FP/LR register */

	ret

	.end
