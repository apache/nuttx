/****************************************************************************
 * arch/ceva/src/xm6/vfork.S
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

	.file		"vfork.S"
	.extern		_up_vfork

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the behavior is
 *   undefined if the process created by vfork() either modifies any data other than
 *   a variable of type pid_t used to store the return value from vfork(), or returns
 *   from the function in which vfork() was called, or calls any other function before
 *   successfully calling _exit() or one of the exec family of functions.
 *
 *   This thin layer implements vfork by simply calling up_vfork() with the vfork()
 *   context as an argument.  The overall sequence is:
 *
 *   1) User code calls vfork().  vfork() collects context information and
 *      transfers control up up_vfork().
 *   2) up_vfork()and calls nxtask_vforksetup().
 *   3) task_vforksetup() allocates and configures the child task's TCB.  This
 *      consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Setup the intput parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state()
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Allocate and initialize the stack
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_vforkstart()
 *   6) nxtask_vforkstart() then executes the child thread.
 *
 * Input Paremeters:
 *   None
 *
 * Return:
 *   Upon successful completion, vfork() returns 0 to the child process and returns
 *   the process ID of the child process to the parent process. Otherwise, -1 is
 *   returned to the parent, no child process is created, and errno is set to
 *   indicate the error.
 *
 ****************************************************************************/

	.text
	.public		_vfork
	.func_start	3 _vfork

_vfork:
	/* Create a stack frame */

	modr		(sp.ui).ui +#-XCPTCONTEXT_SIZE	/* Allocate the structure on the stack */

	/* Save the volatile registers by svcall(SYS_save_context) */

	mov		#SYS_save_context, r0.ui
	mov		sp.ui, r1.ui
	trap		{t0}

	/* Then, call up_vfork(), passing it a pointer to the stack structure */

	mov		sp.ui, r0.ui
	nop
	push		retreg.ui
	callr		#_up_vfork, ?prx.b
	pop		retreg.ui
	nop

	/* Release the stack data and return the value returned by up_vfork */

	modr		(sp.ui).ui +#XCPTCONTEXT_SIZE
	ret		?prx.b

	.func_end	3 _vfork
