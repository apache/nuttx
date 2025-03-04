/****************************************************************************
 * arch/ceva/src/xm6/fork.S
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
#include <nuttx/irq.h>

#include <arch/syscall.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

.IF CONFIG_ARCH_XM6_BUG001
	.EQU		prx pr14
.ELSE
	.EQU		prx pr15
.ENDIF

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"fork.S"
	.extern		up_fork

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fork
 *
 * Description:
 *   The up_fork() function is the base of fork() function that provided in
 *   libc, and fork() is implemented as a wrapper of up_fork() function.
 *   The fork() function has the same effect as posix fork(), except that the behavior is
 *   undefined if the process created by fork() either modifies any data other than
 *   a variable of type pid_t used to store the return value from fork(), or returns
 *   from the function in which fork() was called, or calls any other function before
 *   successfully calling _exit() or one of the exec family of functions.
 *
 *   This thin layer implements fork by simply calling up_fork() with the fork()
 *   context as an argument.  The overall sequence is:
 *
 *   1) User code calls fork().  fork() collects context information and
 *      transfers control up up_fork().
 *   2) ceva_fork()and calls nxtask_forksetup().
 *   3) task_forksetup() allocates and configures the child task's TCB.  This
 *      consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Setup the intput parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state()
 *   4) ceva_fork() provides any additional operating context. ceva_fork must:
 *      - Allocate and initialize the stack
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) ceva_fork() then calls nxtask_forkstart()
 *   6) nxtask_forkstart() then executes the child thread.
 *
 * Input Paremeters:
 *   None
 *
 * Return:
 *   Upon successful completion, fork() returns 0 to the child process and returns
 *   the process ID of the child process to the parent process. Otherwise, -1 is
 *   returned to the parent, no child process is created, and errno is set to
 *   indicate the error.
 *
 ****************************************************************************/

	.text
	.public		up_fork
	.func_start	3 up_fork

up_fork:
	/* Create a stack frame */

	modr		(sp.ui).ui +#-XCPTCONTEXT_SIZE	/* Allocate the structure on the stack */

	/* Save the volatile registers by svcall(SYS_save_context) */

	mov		#SYS_save_context, r0.ui
	mov		sp.ui, r1.ui
	trap		{t0}

	/* Then, call ceva_fork(), passing it a pointer to the stack structure */

	mov		sp.ui, r0.ui
	nop
	push		retreg.ui
	callr		#ceva_fork, ?prx.b
	pop		retreg.ui
	nop

	/* Release the stack data and return the value returned by ceva_fork */

	modr		(sp.ui).ui +#XCPTCONTEXT_SIZE
	ret		?prx.b

	.func_end	3 up_fork
