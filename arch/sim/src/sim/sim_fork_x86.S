/************************************************************************************
 * arch/sim/src/sim/sim_fork_x86.S
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifdef __CYGWIN__
#  define SYMBOL(s) _##s
#elif defined(__ELF__)
#  define SYMBOL(s) s
#else
#  define SYMBOL(s) _##s
#endif

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.file	"fork.S"
	.globl	up_fork

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_fork
 *
 * Description:
 *   The architecture-specific entry point of both of NuttX's cloning
 *   primitives, vfork() and fork().  The caller says which one it is, and
 *   the flag is passed straight through to sim_fork().
 *
 *   On the simulator the caller's context is captured with setjmp() rather
 *   than by hand, and the child re-enters through longjmp() -- which is why
 *   the entry point tests setjmp()'s return value to tell which of the two
 *   returns it is on.
 *
 *   The overall sequence is:
 *
 *   1) User code calls vfork() or fork().  Both are libc wrappers around
 *      this entry point, which collects context information and
 *   2) calls sim_fork(), which calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.  This
 *      consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) sim_fork() provides any additional operating context. sim_fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) sim_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * Input Parameters:
 *   arg0 - true for vfork(), false for fork()
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and returns
 *   the process ID of the child process to the parent process. Otherwise, -1 is
 *   returned to the parent, no child process is created, and errno is set to
 *   indicate the error.
 *
 ************************************************************************************/

	.text

	.globl	SYMBOL(up_fork)
#ifdef __ELF__
	.type	SYMBOL(up_fork), @function
#endif

SYMBOL(up_fork):
	/* %ebx is callee-saved, so it carries the vfork flag across setjmp() */

	push	%ebx
	mov	8(%esp), %ebx

	sub	$XCPTCONTEXT_SIZE, %esp
	push	%esp
	call	SYMBOL(setjmp)

	sub	$1, %eax
	jz	1f

	/* sim_fork(vfork, context).  The context pointer pushed for setjmp()
	 * is still in place, and is the second argument.
	 */

	push	%ebx
	call	SYMBOL(sim_fork)
	add	$4, %esp
1:
	add	$XCPTCONTEXT_SIZE+4, %esp
	pop	%ebx
	ret
#ifdef __ELF__
	.size	SYMBOL(up_fork), . - SYMBOL(up_fork)
#endif
