/************************************************************************************
 * arch/sim/src/sim/sim_fork_x86_64.S
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
	.globl	sim_fork

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
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
 *   2) sim_fork() and calls nxtask_setup_fork().
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
 *   None
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
	sub	$XCPTCONTEXT_SIZE, %rsp
#ifdef CONFIG_SIM_X8664_MICROSOFT
	mov	%rsp, %rcx
#else /* if defined(CONFIG_SIM_X8664_SYSTEMV) */
	mov	%rsp, %rdi
#endif
	call	SYMBOL(setjmp)

	sub	$1, %eax
	jz	child

	call	SYMBOL(sim_fork)
child:
	add	$XCPTCONTEXT_SIZE, %rsp
	ret
#ifdef __ELF__
	.size	SYMBOL(up_fork), . - SYMBOL(up_fork)
#endif
